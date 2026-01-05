#include <Arduino.h>
#include <ESP32Servo.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_srvs/srv/trigger.h>

#define FAST_READ(pin) ((pin)<32 ? ((GPIO.in >> (pin)) & 0x1) : ((GPIO.in1.val >> ((pin)-32)) & 0x1))
#define SGN(a) ((a<0)?-1:1)

int motorPinsA[4] = {15, 5, 4, 2};
int motorPinsB[4] = {16, 23, 14, 19};
int motorPinsEnable[4] = {22, 21, 12, 18};
// motors are ordered from right to left when looking from above and the ESP32's USB port is facing downwards

int encoderPinsB[4] = {26, 32, 34, 39};
int encoderPinsA[4] = {27, 25, 35, 36};

const int middle_servo_pin = 17;
const int left_servo_pin = 13;
const int right_servo_pin = 33;

Servo middle_servo;
Servo left_servo;
Servo right_servo;
int middle_servo_pos_0 = 50;
int middle_servo_pos_1 = 145;

int left_servo_pos_0 = 2;
int left_servo_pos_1 = 145;

int right_servo_pos_0 = 171;
int right_servo_pos_1 = 27;

int minPWM[4] = {80, 80, 80, 80};

// Global variable for isIDLE service
bool isIDLE = true;

// micro-ROS variables
rcl_subscription_t n20_subscriber;
rcl_subscription_t kabech_subscriber;
rcl_subscription_t dawer_subscriber;
rcl_service_t idle_service;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
std_msgs__msg__Int32 n20_msg;
std_msgs__msg__Int32 kabech_msg;
std_msgs__msg__Int32 dawer_msg;
std_srvs__srv__Trigger_Request idle_req;
std_srvs__srv__Trigger_Response idle_res;

// Function declarations
void n20_callback(const void *msgin);
void kabech_callback(const void *msgin);
void dawer_callback(const void *msgin);
void idle_service_callback(const void *req, void *res);

volatile long encoderCount[4] = {0, 0, 0, 0};

float kp[4] = {0.8, 3, 1.5, 3};
float ki[4] = {5, 5, 3, 1.5};
float kd[4] = {0.05, 0.1, 0.01, 0.05};

float integral[4] = {0, 0, 0, 0};
float last_error[4] = {0, 0, 0, 0};

long target_encoder_count[4] = {0, 0, 0, 0};

long upPosition[4] = {0, 0, 0, 0};
long midPosition[4] = {2250, 2210, 2000, 2080};
long downPosition[4] = {8250, 8260, 8000, 8200};
bool motor_homing[4] = {false, false, false, false};
int motor_homing_time[4] = {0, 0, 0, 0};
int saved_encoder_count[4] = {0, 0, 0, 0};

unsigned long last_time = 0;

void IRAM_ATTR encoderA_ISR_0(){ 
    bool A = FAST_READ(encoderPinsA[0]);
    bool B = FAST_READ(encoderPinsB[0]);
    encoderCount[0] += (A == B) ? +1 : -1;
}
void IRAM_ATTR encoderB_ISR_0(){ 
    bool A = FAST_READ(encoderPinsA[0]);
    bool B = FAST_READ(encoderPinsB[0]);
    encoderCount[0] += (A != B) ? +1 : -1;
}
void IRAM_ATTR encoderA_ISR_1(){ 
    bool A = FAST_READ(encoderPinsA[1]);
    bool B = FAST_READ(encoderPinsB[1]);
    encoderCount[1] += (A == B) ? +1 : -1;
}
void IRAM_ATTR encoderB_ISR_1(){ 
    bool A = FAST_READ(encoderPinsA[1]);
    bool B = FAST_READ(encoderPinsB[1]);
    encoderCount[1] += (A != B) ? +1 : -1;
}
void IRAM_ATTR encoderA_ISR_2(){ 
    bool A = FAST_READ(encoderPinsA[2]);
    bool B = FAST_READ(encoderPinsB[2]);
    encoderCount[2] += (A == B) ? +1 : -1;
}
void IRAM_ATTR encoderB_ISR_2(){ 
    bool A = FAST_READ(encoderPinsA[2]);
    bool B = FAST_READ(encoderPinsB[2]);
    encoderCount[2] += (A != B) ? +1 : -1;
}
void IRAM_ATTR encoderA_ISR_3(){ 
    bool A = FAST_READ(encoderPinsA[3]);
    bool B = FAST_READ(encoderPinsB[3]);
    encoderCount[3] += (A == B) ? +1 : -1;
}
void IRAM_ATTR encoderB_ISR_3(){ 
    bool A = FAST_READ(encoderPinsA[3]);
    bool B = FAST_READ(encoderPinsB[3]);
    encoderCount[3] += (A != B) ? +1 : -1;
}

void send_values(float s, float ts, long pos, long tgt){
    Serial.write((uint8_t)0xAA); Serial.write((uint8_t*)&s, sizeof(s));
    Serial.write((uint8_t)0xAB); Serial.write((uint8_t*)&ts, sizeof(ts));
    Serial.write((uint8_t)0xAC); Serial.write((uint8_t*)&pos, sizeof(pos));
    Serial.write((uint8_t)0xAD); Serial.write((uint8_t*)&tgt, sizeof(tgt));
}

void setupServos(){
    middle_servo.setPeriodHertz(50);
    middle_servo.attach(middle_servo_pin);
    middle_servo.write(middle_servo_pos_0);
    left_servo.setPeriodHertz(50);
    left_servo.attach(left_servo_pin);
    left_servo.write(left_servo_pos_0);
    right_servo.setPeriodHertz(50);
    right_servo.attach(right_servo_pin);
    right_servo.write(right_servo_pos_0);
}

void setMotor(int pwm, int motorIndex){  
    if(abs(pwm)<minPWM[motorIndex]){
    pwm = 0;
    }
    pwm = constrain(pwm , -255 , 255);
    analogWrite(motorPinsEnable[motorIndex], abs(pwm));
    if (pwm > 0){
        digitalWrite(motorPinsA[motorIndex], 1);
        digitalWrite(motorPinsB[motorIndex], 0);
    }
    else if (pwm < 0){
        digitalWrite(motorPinsA[motorIndex], 0);
        digitalWrite(motorPinsB[motorIndex], 1);
    }else{
        digitalWrite(motorPinsEnable[motorIndex], 1);
        digitalWrite(motorPinsA[motorIndex], 0);
        digitalWrite(motorPinsB[motorIndex], 0);
    }
}

void applyPositionPID(float dt, int motorIndex){
    if(target_encoder_count[motorIndex] != upPosition[motorIndex] ){
        float error = target_encoder_count[motorIndex] - encoderCount[motorIndex];
        integral[motorIndex] += error * dt;
        if(SGN(last_error[motorIndex]) != SGN(error)){
        integral[motorIndex] = 0;
        }
        integral[motorIndex] = constrain(integral[motorIndex], -500.0f, 500.0f);
        float derivative = (error - last_error[motorIndex]) / dt;
        float speedCmd = kp[motorIndex] * error + ki[motorIndex] * integral[motorIndex] + kd[motorIndex] * derivative;

        last_error[motorIndex] = error;
        setMotor((int)speedCmd, motorIndex);
    }else{
        if (!motor_homing[motorIndex]){
            setMotor(-255, motorIndex);
            motor_homing[motorIndex] = true;
            saved_encoder_count[motorIndex] = encoderCount[motorIndex];
            motor_homing_time[motorIndex] = millis();
        }else if (millis() - motor_homing_time[motorIndex] > 100){
            if (abs(encoderCount[motorIndex] - saved_encoder_count[motorIndex]) == 0){
                motor_homing[motorIndex] = false;
                setMotor(0, motorIndex);
                encoderCount[motorIndex] = upPosition[motorIndex];
                target_encoder_count[motorIndex] = upPosition[motorIndex] + 10;
            }else{
                saved_encoder_count[motorIndex] = encoderCount[motorIndex];
            }
            motor_homing_time[motorIndex] = millis();
        }
    }
}

void ba33ed_3la_ba3dhou(){
    middle_servo.write(middle_servo_pos_1);
}

void lemm_l_ba3dhou(){
    middle_servo.write(middle_servo_pos_0);
}

void estwi_l_crates(){
    left_servo.write(left_servo_pos_1);
    right_servo.write(right_servo_pos_1);
}

void khabbi_l_kabecha(){
    left_servo.write(left_servo_pos_0);
    right_servo.write(right_servo_pos_0);
}

// Callback function for /forward/n20 topic
void n20_callback(const void *msgin) {
    const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;
    int value = msg->data;

}

// Callback function for /forward/kabech topic
void kabech_callback(const void *msgin) {
    const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;
    int value = msg->data;
}

// Callback function for /forward/dawer topic
void dawer_callback(const void *msgin) {
    const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;
    int value = msg->data;
}
void find_home(){
    for(int i=0; i<4; i++){
        setMotor(-255, i);
    }
    int motors_at_home = 0;
    int saved_encoder_counts[4] = {encoderCount[0], encoderCount[1], encoderCount[2], encoderCount[3]};
    bool motor_homed[4] = {false, false, false, false};
    delay(500);
    while(motors_at_home < 4){
        delay(200);
        for(int i=0; i<4; i++){
            if (!motor_homed[i] && abs(encoderCount[i] - saved_encoder_counts[i]) == 0){
                motors_at_home++;
                motor_homed[i] = true;
                setMotor(0, i);
                encoderCount[i] = upPosition[i];
                Serial.println("Motor " + String(i) + " homed.");
            }else{
                saved_encoder_counts[i] = encoderCount[i];
            }
        }
    }
}
bool is_idle(){
    bool idle = true;
    for(int i=0; i<4; i++){
        if (abs(target_encoder_count[i] - encoderCount[i]) > 35){
            idle = false;
            break;
        }
    }
    return idle;
}
void apply_command(String command){
    if (command[0] == 'u' || command[0] == 'm' || command[0] == 'd' || command[0] == 'p'){
        for(int i=0; i<4; i++){
            if (command[i] == 'u'){
                target_encoder_count[i] = upPosition[i];
            }else if (command[i] == 'm'){
                target_encoder_count[i] = midPosition[i];
            }else if (command[i] == 'd'){
                target_encoder_count[i] = downPosition[i];
            }
        }
    }else{
        target_encoder_count[(int)command[0]-48] = command.substring(1).toInt();
    }
}

// Service callback for /front/isIDLE service
void idle_service_callback(const void *req, void *res) {
    std_srvs__srv__Trigger_Response *response = (std_srvs__srv__Trigger_Response *)res;
    
    // Return the isIDLE status
    response->success = isIDLE;
    
    // Set response message
    if (isIDLE) {
        strcpy(response->message.data, "System is IDLE");
    } else {
        strcpy(response->message.data, "System is BUSY");
    }
    response->message.size = strlen(response->message.data);
    response->message.capacity = response->message.size + 1;
}

void setup(){
    Serial.begin(115200);
    Serial.println("Starting ESP32 back Control Node...");
    set_microros_transports();

    for(int i=0; i<4; i++){
        pinMode(encoderPinsA[i], INPUT_PULLUP);
        pinMode(encoderPinsB[i], INPUT_PULLUP);
        pinMode(motorPinsA[i], OUTPUT);
        pinMode(motorPinsB[i], OUTPUT);
    }

    attachInterrupt(digitalPinToInterrupt(encoderPinsA[0]), encoderA_ISR_0, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderPinsB[0]), encoderB_ISR_0, CHANGE);

    attachInterrupt(digitalPinToInterrupt(encoderPinsA[1]), encoderA_ISR_1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderPinsB[1]), encoderB_ISR_1, CHANGE);

    attachInterrupt(digitalPinToInterrupt(encoderPinsA[2]), encoderA_ISR_2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderPinsB[2]), encoderB_ISR_2, CHANGE);

    attachInterrupt(digitalPinToInterrupt(encoderPinsA[3]), encoderA_ISR_3, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderPinsB[3]), encoderB_ISR_3, CHANGE);

    delay(2000);
    setupServos();
    /*
    // Initialize micro-ROS with error checking and retry
    Serial.println("Waiting for micro-ROS agent...");
    delay(3000);
    
    allocator = rcl_get_default_allocator();
    
    // Retry connection up to 10 times
    rcl_ret_t ret;
    int retry_count = 0;
    do {
        ret = rclc_support_init(&support, 0, NULL, &allocator);
        if (ret != RCL_RET_OK) {
            Serial.println("Failed to connect to micro-ROS agent, retrying...");
            delay(1000);
            retry_count++;
        }
    } while (ret != RCL_RET_OK && retry_count < 10);
    
    if (ret != RCL_RET_OK) {
        Serial.println("ERROR: Failed to initialize micro-ROS!");
        while(1) { delay(100); }
    }
    Serial.println("✓ Support initialized");
    
    ret = rclc_node_init_default(&node, "esp32_front_node", "", &support);
    if (ret != RCL_RET_OK) {
        Serial.println("ERROR: Failed to init node!");
        while(1) { delay(100); }
    }
    Serial.println("✓ Node created");

    // Create subscriptions
    ret = rclc_subscription_init_default(
        &n20_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "/front/n20");
    if (ret != RCL_RET_OK) {
        Serial.println("ERROR: Failed to create n20 subscription!");
    } else {
        Serial.println("✓ N20 subscription created");
    }

    ret = rclc_subscription_init_default(
        &kabech_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "/front/kabech");
    if (ret != RCL_RET_OK) {
        Serial.println("ERROR: Failed to create kabech subscription!");
    } else {
        Serial.println("✓ Kabech subscription created");
    }

    ret = rclc_subscription_init_default(
        &dawer_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "/front/dawer");
    if (ret != RCL_RET_OK) {
        Serial.println("ERROR: Failed to create dawer subscription!");
    } else {
        Serial.println("✓ Dawer subscription created");
    }

    // Create isIDLE service
    ret = rclc_service_init_default(
        &idle_service,
        &node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger),
        "/front/isIDLE");
    if (ret != RCL_RET_OK) {
        Serial.println("ERROR: Failed to create isIDLE service!");
    } else {
        Serial.println("✓ isIDLE service created");
    }

    // Create executor (4 handles: 3 subscriptions + 1 service)
    ret = rclc_executor_init(&executor, &support.context, 4, &allocator);
    if (ret != RCL_RET_OK) {
        Serial.println("ERROR: Failed to init executor!");
    } else {
        Serial.println("✓ Executor initialized");
    };
    rclc_executor_add_subscription(&executor, &n20_subscriber, &n20_msg,
                                    &n20_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &kabech_subscriber, &kabech_msg,
                                    &kabech_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &dawer_subscriber, &dawer_msg,
                                    &dawer_callback, ON_NEW_DATA);
    rclc_executor_add_service(&executor, &idle_service, &idle_req, &idle_res,
                              &idle_service_callback);
    
    Serial.println("✓✓✓ ESP32 Back Node Ready! ✓✓✓");
    */
    last_time = millis()-10;
    // target_encoder_count[0] = -6000;
    // target_encoder_count[1] = -6050;
    // target_encoder_count[2] = -6000;
    // target_encoder_count[3] = -6120;
    Serial.println("tryna home");
    find_home();
    Serial.println("Homing complete.");
    for(int i=0; i<4; i++){
        setMotor(0, i);
    }
    apply_command("mmmm");
}

void loop(){
    unsigned long currentTime = millis();
    float dt = (currentTime - last_time) / 1000.0;
    last_time = currentTime;
    
    //Spin micro-ROS executor
    //rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    
    for (int i=0; i<4; i++){
        applyPositionPID(dt, i);
    }
    delay(10);
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        if (command[0] == 'u' || command[0] == 'm' || command[0] == 'd'){
            for(int i=0; i<4; i++){
                if (command[i] == 'u'){
                    target_encoder_count[i] = upPosition[i];
                }else if (command[i] == 'm'){
                    target_encoder_count[i] = midPosition[i];
                }else if (command[i] == 'd'){
                    target_encoder_count[i] = downPosition[i];
                }
            }
        }else if (command[0] == 'k'){
            estwi_l_crates();
        }else if (command[0] == 'l'){
            khabbi_l_kabecha();
        }else if (command[0] == 'b'){
            ba33ed_3la_ba3dhou();
        }else if (command[0] == 's'){
            lemm_l_ba3dhou();
        }else{
            target_encoder_count[(int)command[0]-48] = command.substring(1).toInt();
        }
    }
}
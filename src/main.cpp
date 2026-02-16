#include <Arduino.h>
#include <ESP32Servo.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/string.h>
#include <std_srvs/srv/trigger.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <rmw_microros/rmw_microros.h>

#define FAST_READ(pin) ((pin)<32 ? ((GPIO.in >> (pin)) & 0x1) : ((GPIO.in1.val >> ((pin)-32)) & 0x1))
#define SGN(a) ((a<0)?-1:1)

// Agent status enum
typedef enum {
    AGENT_WAIT = 0,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
} AgentStatus;

AgentStatus agent_status = AGENT_WAIT;

// FreeRTOS Task Handles
TaskHandle_t microROSTaskHandle = NULL;
TaskHandle_t pidControlTaskHandle = NULL;

// Mutex for shared resources
SemaphoreHandle_t encoderMutex;
SemaphoreHandle_t targetMutex;
SemaphoreHandle_t servoMutex;

int motorPinsA[4] = {2, 4, 5, 15};
int motorPinsB[4] = {19, 14, 23, 16};
int motorPinsEnable[4] = {18, 12, 21, 22};
// motors are ordered from right to left when looking from above and the ESP32's USB port is facing downwards

int encoderPinsB[4] = {39, 34, 32, 26};
int encoderPinsA[4] = {36 ,35 ,25 ,27};

const int middle_servo_pin = 17;
const int left_servo_pin = 13;
const int right_servo_pin = 33;

Servo middle_servo;
Servo left_servo;
Servo right_servo;
int middle_servo_pos_0 = 40;
int middle_servo_pos_1 = 152;

int left_servo_pos_0 = 14;
int left_servo_pos_1 = 150;

int right_servo_pos_0 = 155;
int right_servo_pos_1 = 19;

int minPWM[4] = {80, 80, 80, 80};

// Global variable for isIDLE service
bool isIDLE = true;
bool setup_completed = false;
bool homed = false;

// micro-ROS variables
rcl_subscription_t n20_subscriber;
rcl_subscription_t kabech_subscriber;
rcl_subscription_t dawer_subscriber;
rcl_publisher_t command_publisher;
rcl_service_t idle_service;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
std_msgs__msg__Int32 n20_msg;
std_msgs__msg__Int32 kabech_msg;
std_msgs__msg__Int32 dawer_msg;
std_msgs__msg__String command_msg;
std_srvs__srv__Trigger_Request idle_req;
std_srvs__srv__Trigger_Response idle_res;

// Function declarations
void n20_callback(const void *msgin);
void kabech_callback(const void *msgin);
void dawer_callback(const void *msgin);
void idle_service_callback(const void *req, void *res);

volatile long encoderCount[4] = {0, 0, 0, 0};

float kp[4] = {3, 3, 1.5, 3};
float ki[4] = {5, 5, 3, 1.5};
float kd[4] = {0.05, 0.1, 0.01, 0.05};

float integral[4] = {0, 0, 0, 0};
float last_error[4] = {0, 0, 0, 0};

long target_encoder_count[4] = {0, 0, 0, 0};
int pwm_output[4] = {0, 0, 0, 0};

long upPosition[4] = {0, 0, 0, 0};
long midPosition[4] = {2250, 2100, 2000, 2080};
long preparePosition[4] = {7250, 6700, 7000, 7080};
long downPosition[4] = {8500, 8100, 8500, 8200};
bool motor_homing[4] = {false, false, false, false};
int motor_homing_time[4] = {0, 0, 0, 0};
int saved_encoder_count[4] = {0, 0, 0, 0};
int first_saved_encoder_count[4] = {0, 0, 0, 0};

bool going_down = false;
int before_going_up_encoder_count[4] = {0, 0, 0, 0};

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
    pwm_output[motorIndex] = pwm;
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
        digitalWrite(motorPinsA[motorIndex], 1);
        digitalWrite(motorPinsB[motorIndex], 1);
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
            first_saved_encoder_count[motorIndex] = encoderCount[motorIndex];
            motor_homing_time[motorIndex] = millis();
        }else if (millis() - motor_homing_time[motorIndex] > 350 && (encoderCount[motorIndex] < first_saved_encoder_count[motorIndex]/2) ){
            if (abs(encoderCount[motorIndex] - saved_encoder_count[motorIndex]) == 0){
                motor_homing[motorIndex] = false;
                setMotor(0, motorIndex);
                encoderCount[motorIndex] = upPosition[motorIndex];
                target_encoder_count[motorIndex] = upPosition[motorIndex] + 40;
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
void apply_command(String command){
    for(int i=0; i<4; i++){
        motor_homing[i] = false;
    }
    xSemaphoreTake(targetMutex, portMAX_DELAY);
    if (command[0] == 'p' || command[0] == 'u' || command[0] == 'm' || command[0] == 'd'){
        for(int i=0; i<4; i++){
            if (command[i] == 'u'){
                target_encoder_count[i] = upPosition[i];
            }else if (command[i] == 'm'){
                target_encoder_count[i] = midPosition[i];
            }else if (command[i] == 'p'){
                target_encoder_count[i] = preparePosition[i];
            }else if (command[i] == 'd'){
                target_encoder_count[i] = downPosition[i];
            }
        }
    }else{
        target_encoder_count[(int)command[0]-48] = command.substring(1).toInt();
    }
    xSemaphoreGive(targetMutex);
}
void find_home(){
    homed = false;
    for(int i=0; i<4; i++){
        setMotor(-255, i);
    }
    int motors_at_home = 0;
    int saved_encoder_counts[4] = {encoderCount[0], encoderCount[1], encoderCount[2], encoderCount[3]};
    bool motor_homed[4] = {false, false, false, false};
    delay(500);
    while(motors_at_home < 4){
        delay(350);
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
    homed = true;
}

// Callback function for /forward/n20 topic
void n20_callback(const void *msgin) {
    const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;
    int value = msg->data;
    if (!value){
        find_home();
        apply_command("mmmm");
        return;
    }
    // 1 means down, 2 means middle, and 3 means up, ordered from MSB to LSB
    String command = "";
    going_down = false;
    for (int i=0; i<4; i++){
        int bit = value % 10;
        if (bit == 1){
            command = 'd' + command; // down
            going_down = true;
        }else if (bit == 2){
            command = 'p' + command; // prepare
        }else if (bit == 3){
            command = 'm' + command; // middle
        }else{
            command = 'u' + command; // up
        }
        value /= 10;
    }
    apply_command(command);

    String s = "new command :" + command;
    
    command_msg.data.data = (char*)malloc(s.length() + 1);
    strcpy(command_msg.data.data, s.c_str());
    command_msg.data.size = s.length();
    command_msg.data.capacity = s.length() + 1;
    
    // Publish the command string
    rcl_publish(&command_publisher, &command_msg, NULL);
    
    // Free allocated memory
    free(command_msg.data.data);
    command_msg.data.data = NULL;
    for(int i=0; i<20; i++){
        String s =
            String(pwm_output[0]) + " " +
            String(pwm_output[1]) + " " +
            String(pwm_output[2]) + " " +
            String(pwm_output[3]) + " counts:" +
            String(encoderCount[0]) + " " +
            String(encoderCount[1]) + " " +
            String(encoderCount[2]) + " " +
            String(encoderCount[3]);;
        
        command_msg.data.data = (char*)malloc(s.length() + 1);
        strcpy(command_msg.data.data, s.c_str());
        command_msg.data.size = s.length();
        command_msg.data.capacity = s.length() + 1;
        
        // Publish the command string
        rcl_publish(&command_publisher, &command_msg, NULL);
        
        // Free allocated memory
        free(command_msg.data.data);
        command_msg.data.data = NULL;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Callback function for /forward/kabech topic
void kabech_callback(const void *msgin) {
    const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;
    int value = msg->data;
    if (value == 1){
        estwi_l_crates();
    }else{
        khabbi_l_kabecha();
    }
}

// Callback function for /forward/dawer topic
void dawer_callback(const void *msgin) {
    const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;
    int value = msg->data;
    if (value == 1){
        ba33ed_3la_ba3dhou();
    }else{
        lemm_l_ba3dhou();
    }
}

bool is_idle(int tolerance=50){
    bool idle = true;
    for(int i=0; i<4; i++){
        if (abs(target_encoder_count[i] - encoderCount[i]) > tolerance){
            idle = false;
            break;
        }
    }
    return idle;
}
// Service callback for /back/isIDLE service
void idle_service_callback(const void *req, void *res) {
    std_srvs__srv__Trigger_Response *response = (std_srvs__srv__Trigger_Response *)res;
    
    // Check if system is idle
    bool system_idle = homed && (( going_down && is_idle() ) || (!going_down && is_idle(7000)));
    
    // Set response success field
    response->success = system_idle;
    
    // Properly allocate and assign the response message using micro-ROS string utilities
    const char* msg = system_idle ? "System is IDLE" : "System is BUSY";
    
    // Allocate memory for the string
    response->message.data = (char*)malloc(strlen(msg) + 1);
    if (response->message.data != NULL) {
        strcpy(response->message.data, msg);
        response->message.size = strlen(msg);
        response->message.capacity = strlen(msg) + 1;
    } else {
        // Fallback if allocation fails
        response->message.data = (char*)"";
        response->message.size = 0;
        response->message.capacity = 0;
    }
}

void test_with_serial(){
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        if (command[0] == 'p' || command[0] == 'u' || command[0] == 'm' || command[0] == 'd'){
            for(int i=0; i<4; i++){
                if (command[i] == 'u'){
                    target_encoder_count[i] = upPosition[i];
                }else if (command[i] == 'm'){
                    target_encoder_count[i] = midPosition[i];
                }else if (command[i] == 'p'){
                    target_encoder_count[i] = preparePosition[i];
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
        }else if (command[0] == 'c') {
            // format: c<servo_id><angle>
            // example: // example: c130  -> right servo to 90°
            //          c260  -> left servo to 45°
            //          c3120 -> middle servo to 120°
            // Note: The servo IDs are relative to the face you are testing from, not according to the right and left of the robot itself

            int servo_id = command[1] - '1';   // '1'->0, '2'->1, '3'->2

            if (servo_id >= 0 && servo_id < 3) {
                int angle = command.substring(2).toInt();

                // OPTION 1: using array
                if (servo_id == 1){
                    left_servo.write(angle);
                    Serial.print("moving left servo to angle ");
                    Serial.println(angle);
                }else if (servo_id == 2){
                    middle_servo.write(angle);
                    Serial.print("moving middle servo to angle ");
                    Serial.println(angle);
                }else{
                    right_servo.write(angle);
                    Serial.print("moving right servo to angle ");
                    Serial.println(angle);
                }

                // OPTION 2: using function
                // setServoAngle(servo_id, angle);
            }
        }else{
            target_encoder_count[(int)command[0]-48] = command.substring(1).toInt();
        }
    }
}

// ============================================================================
// Agent Connection Management Functions
// ============================================================================
bool pingAgent() {
    // Ping the agent to check if it's still alive
    rcl_ret_t ping_result = rmw_uros_ping_agent(100, 1); // 100ms timeout, 1 attempt
    return ping_result == RCL_RET_OK;
}

void cleanupMicroROS() {
    Serial.println("Cleaning up micro-ROS resources...");
    
    // Clean up executor
    rclc_executor_fini(&executor);
    
    // Clean up service
    rcl_service_fini(&idle_service, &node);
    
    // Clean up publisher
    rcl_publisher_fini(&command_publisher, &node);
    
    // Clean up subscriptions
    rcl_subscription_fini(&n20_subscriber, &node);
    rcl_subscription_fini(&kabech_subscriber, &node);
    rcl_subscription_fini(&dawer_subscriber, &node);
    
    // Clean up node and support
    rcl_node_fini(&node);
    rclc_support_fini(&support);
    
    Serial.println("✓ Cleanup complete");
}

bool initializeMicroROS() {
    Serial.println("Initializing micro-ROS...");
    
    allocator = rcl_get_default_allocator();
    rcl_ret_t ret = rclc_support_init(&support, 0, NULL, &allocator);
    
    if (ret != RCL_RET_OK) {
        Serial.println("✗ Failed to initialize support");
        return false;
    }
    Serial.println("✓ Support initialized");
    
    ret = rclc_node_init_default(&node, "esp32_front_node", "", &support);
    if (ret != RCL_RET_OK) {
        Serial.println("✗ Failed to init node");
        rclc_support_fini(&support);
        return false;
    }
    Serial.println("✓ Node created");

    // Create QoS profile with reliable settings
    rcl_subscription_options_t sub_opts = rcl_subscription_get_default_options();
    sub_opts.qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    sub_opts.qos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    sub_opts.qos.depth = 1;

    // Create subscriptions with reliable QoS
    ret = rclc_subscription_init(
        &n20_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "/front/n20",
        &sub_opts.qos);
    if (ret != RCL_RET_OK) {
        Serial.println("✗ Failed to create n20 subscription");
        rcl_node_fini(&node);
        rclc_support_fini(&support);
        return false;
    }
    Serial.println("✓ N20 subscription created");

    ret = rclc_subscription_init(
        &kabech_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "/front/kabech",
        &sub_opts.qos);
    if (ret != RCL_RET_OK) {
        Serial.println("✗ Failed to create kabech subscription");
        rcl_subscription_fini(&n20_subscriber, &node);
        rcl_node_fini(&node);
        rclc_support_fini(&support);
        return false;
    }
    Serial.println("✓ Kabech subscription created");

    ret = rclc_subscription_init(
        &dawer_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "/front/dawer",
        &sub_opts.qos);
    if (ret != RCL_RET_OK) {
        Serial.println("✗ Failed to create dawer subscription");
        rcl_subscription_fini(&n20_subscriber, &node);
        rcl_subscription_fini(&kabech_subscriber, &node);
        rcl_node_fini(&node);
        rclc_support_fini(&support);
        return false;
    }
    Serial.println("✓ Dawer subscription created");

    // Create publisher with reliable QoS
    rcl_publisher_options_t pub_opts = rcl_publisher_get_default_options();
    pub_opts.qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    pub_opts.qos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    pub_opts.qos.depth = 1;

    ret = rclc_publisher_init(
        &command_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "/back/command",
        &pub_opts.qos);
    if (ret != RCL_RET_OK) {
        Serial.println("✗ Failed to create command publisher");
        rcl_subscription_fini(&n20_subscriber, &node);
        rcl_subscription_fini(&kabech_subscriber, &node);
        rcl_subscription_fini(&dawer_subscriber, &node);
        rcl_node_fini(&node);
        rclc_support_fini(&support);
        return false;
    }
    Serial.println("✓ Command publisher created");

    // Create service options with reliable QoS
    rcl_service_options_t svc_opts = rcl_service_get_default_options();
    svc_opts.qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    svc_opts.qos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    svc_opts.qos.depth = 1;

    // Create isIDLE service with reliable QoS
    ret = rclc_service_init(
        &idle_service,
        &node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger),
        "/front/isIDLE",
        &svc_opts.qos);
    if (ret != RCL_RET_OK) {
        Serial.println("✗ Failed to create isIDLE service");
        rcl_subscription_fini(&n20_subscriber, &node);
        rcl_subscription_fini(&kabech_subscriber, &node);
        rcl_subscription_fini(&dawer_subscriber, &node);
        rcl_node_fini(&node);
        rclc_support_fini(&support);
        return false;
    }
    Serial.println("✓ isIDLE service created");

    // Create executor (4 handles: 3 subscriptions + 1 service)
    ret = rclc_executor_init(&executor, &support.context, 4, &allocator);
    if (ret != RCL_RET_OK) {
        Serial.println("✗ Failed to init executor");
        rcl_service_fini(&idle_service, &node);
        rcl_subscription_fini(&n20_subscriber, &node);
        rcl_subscription_fini(&kabech_subscriber, &node);
        rcl_subscription_fini(&dawer_subscriber, &node);
        rcl_node_fini(&node);
        rclc_support_fini(&support);
        return false;
    }
    Serial.println("✓ Executor initialized");
    
    rclc_executor_add_subscription(&executor, &n20_subscriber, &n20_msg,
                                    &n20_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &kabech_subscriber, &kabech_msg,
                                    &kabech_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &dawer_subscriber, &dawer_msg,
                                    &dawer_callback, ON_NEW_DATA);
    rclc_executor_add_service(&executor, &idle_service, &idle_req, &idle_res,
                              &idle_service_callback);
    
    Serial.println("✓✓✓ ESP32 Front Node Ready! ✓✓✓");
    return true;
}

// ============================================================================
// FreeRTOS Task: micro-ROS Communication with Advanced Connection Management
// ============================================================================
void microROSTask(void *parameter) {
    Serial.println("micro-ROS Task started on Core: " + String(xPortGetCoreID()));
    
    unsigned long last_ping_time = 0;
    const unsigned long PING_INTERVAL = 500; // Ping every 0.5 second
    const unsigned long RECONNECT_DELAY = 500; // Wait 0.5 second before reconnect
    
    agent_status = AGENT_WAIT;
    
    while (true) {
        switch (agent_status) {
            case AGENT_WAIT:
            case AGENT_DISCONNECTED:
                Serial.println("[STATUS] Waiting for agent...");
                agent_status = AGENT_WAIT;
                
                // Check if agent is available
                if (pingAgent()) {
                    Serial.println("[STATUS] Agent detected! Attempting connection...");
                    agent_status = AGENT_AVAILABLE;
                } else {
                    vTaskDelay(pdMS_TO_TICKS(RECONNECT_DELAY));
                }
                break;
                
            case AGENT_AVAILABLE:
                // Try to initialize micro-ROS
                if (initializeMicroROS()) {
                    agent_status = AGENT_CONNECTED;
                    last_ping_time = millis();
                    Serial.println("[STATUS] Successfully connected to agent");
                } else {
                    Serial.println("[STATUS] Connection failed, retrying...");
                    agent_status = AGENT_WAIT;
                    vTaskDelay(pdMS_TO_TICKS(RECONNECT_DELAY));
                }
                break;
                
            case AGENT_CONNECTED:
                // Spin executor to process messages
                rcl_ret_t ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(20));
                
                // Periodic ping to check agent health
                if (millis() - last_ping_time > PING_INTERVAL) {
                    if (!pingAgent()) {
                        Serial.println("[STATUS] Agent ping failed! Connection lost.");
                        cleanupMicroROS();
                        agent_status = AGENT_DISCONNECTED;
                    } else {
                        // Uncomment for verbose ping status
                        // Serial.println("[STATUS] Agent ping OK");
                    }
                    last_ping_time = millis();
                }
                
                vTaskDelay(pdMS_TO_TICKS(20));
                break;
        }
        
        // Small delay for non-connected states is already handled in each case
        if (agent_status == AGENT_CONNECTED) {
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }
}

// ============================================================================
// FreeRTOS Task: PID Control Loop
// ============================================================================
void pidControlTask(void *parameter) {
    Serial.println("PID Control Task started on Core: " + String(xPortGetCoreID()));
    
    unsigned long last_time = millis();
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // 100Hz PID update
    
    while (true) {
        unsigned long currentTime = millis();
        float dt = (currentTime - last_time) / 1000.0;
        last_time = currentTime;
        
        // Apply PID control for all 4 motors
        if (homed){
            xSemaphoreTake(targetMutex, portMAX_DELAY);
            for (int i = 0; i < 4; i++) {
                applyPositionPID(dt, i);
            }
            xSemaphoreGive(targetMutex);
        }
         //test_with_serial();
        // Delay until next cycle
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void microros_setup(){
    Serial.println("Starting ESP32 back Control Node...");
    
    // Create FreeRTOS mutexes for shared resource protection
   
    
    if (encoderMutex == NULL || targetMutex == NULL || servoMutex == NULL) {
        Serial.println("ERROR: Failed to create mutexes!");
        while(1) { delay(100); }
    }
    
    set_microros_transports();
    Serial.println("✓ micro-ROS transport initialized");
    Serial.println("Connection will be handled by microROSTask...");
}

void setup(){
    Serial.begin(115200);
 encoderMutex = xSemaphoreCreateMutex();
    targetMutex = xSemaphoreCreateMutex();
    servoMutex = xSemaphoreCreateMutex();
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

    microros_setup();
    
    Serial.println("tryna home");
    find_home();
    Serial.println("Homing complete.");
    apply_command("mmmm");
    
    // ========================================================================
    // Create FreeRTOS Tasks
    // ========================================================================
    
    // Create micro-ROS task on Core 0 (higher priority)
    xTaskCreatePinnedToCore(
        microROSTask,           // Task function
        "microROSTask",         // Task name
        8192,                   // Stack size (bytes)
        NULL,                   // Parameters
        2,                      // Priority (0-24, higher number = higher priority)
        &microROSTaskHandle,    // Task handle
        0                      // Core ID (0 or 1)
    );
    
    // Create PID control task on Core 1 (medium priority)
    xTaskCreatePinnedToCore(
        pidControlTask,         // Task function
        "pidControlTask",       // Task name
        4096,                   // Stack size (bytes)
        NULL,                   // Parameters
        1,                      // Priority
        &pidControlTaskHandle,  // Task handle
        1                       // Core ID (0 or 1)
    );
    
    if (microROSTaskHandle == NULL || pidControlTaskHandle == NULL) {
        Serial.println("ERROR: Failed to create FreeRTOS tasks!");
        while(1) { delay(100); }
    }
    
    Serial.println("✓ FreeRTOS tasks created successfully");
    Serial.println("✓ micro-ROS task on Core 0 (Priority 2)");
    Serial.println("✓ PID Control task on Core 1 (Priority 1)");
    Serial.println("=== System ready ===");
    setup_completed = true;
}
void loop(){
    // Empty loop - all work is done by FreeRTOS tasks
    // The loop() function is still called but doesn't need to do anything
    vTaskDelay(pdMS_TO_TICKS(1000)); // Yield to FreeRTOS scheduler
}
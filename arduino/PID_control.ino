#include <ros.h>
#include <std_msgs/Int16.h>
#include <lane_follower/MotorPWM_msg.h>

// Set the serial port in the Arduino IDE to the correct port for your board(Arduino Mega).
#define AIN1 6 //Left wheel control 1
#define AIN2 5 // Left wheel control 2
#define PWMA 4 // Left wheel PWM
#define BIN1 10 // Right wheel control 1
#define BIN2 9 // Right wheel control 2
#define PWMB 8 // Right wheel PWM
#define STBY 7 // Steering control

// Encoder pins setup
#define Pin_Encoder_Right_A 2
#define Pin_Encoder_Right_B 3
#define Pin_Encoder_Left_A 20
#define Pin_Encoder_Left_B 21

// Encoder to count the number of pulses
volatile long theta_left = 0, theta_right = 0;
long prev_theta_right = 0, prev_theta_left = 0;

// Time counter
unsigned long currentMillis = 0;
long previousMillis = 0;
float interval = 100; // control frequency 10Hz

// PWM values
int pwm_left = 0, pwm_right = 0;
int target_pwm_left = 0, target_pwm_right = 0;

// Speed values
float speed_left = 0, speed_right = 0;
float speed_left_filtered = 0, speed_right_filtered = 0;

// PID parameters
float kp = 0.5; // Proportional gain
float ki = 0.2; // Integral gain
float kd = 0.05; // Derivative gain

float error_left_prev = 0, error_right_prev = 0;
float integral_left = 0, integral_right = 0;

// Build ros node
ros::NodeHandle nh;

// Define callback functions
void motorPwmCallback(const lane_follower::MotorPWM_msg& msg){
    target_pwm_left = msg.left_pwm;
    target_pwm_right = msg.right_pwm;
    Serial.print("Received left PWM: ");
    Serial.print(target_pwm_left);
    Serial.print(", right PWM: ");
    Serial.println(target_pwm_right);
}

// Build subscriber
// And <lane_follower::MotorPWM_msg> to specify which package the message is from
ros::Subscriber<lane_follower::MotorPWM_msg> motorPWMSub("/motor_pwm", &motorPwmCallback);

// Build publisher to publish the pwm values
std_msgs::Int16 left_encoder_msg;
std_msgs::Int16 right_encoder_msg;
ros::Publisher left_encoder_pub("left_encoder", &left_encoder_msg);
ros::Publisher right_encoder_pub("right_encoder", &right_encoder_msg);

void setup(){
    // Setup the motor and encoder pins
    initMotor();
    initEncoder();

    // Initialize the ros node
    nh.initNode();
    nh.subscribe(motorPWMSub);
    nh.advertise(left_encoder_pub);
    nh.advertise(right_encoder_pub);
}

void loop(){
    currentMillis = millis();

    // In specific periof, do the PID control
    if(currentMillis-previousMillis >= interval){
        // Calculate the speed(encoder count change / time interval)
        speed_left = (theta_left-prev_theta_left) * 1000/ interval; // *1000 is to convert to s 
        speed_right = (theta_right-prev_theta_right) * 1000/ interval;// *1000 is to convert to s

        // Filter the speed to reduce noise
        speed_left_filtered = 0.7 * speed_left_filtered + 0.3 * speed_left;
        speed_right_filtered = 0.7 * speed_right_filtered + 0.3 * speed_right;

        // Remain the previous encoder count
        prev_theta_left = theta_left;
        prev_theta_right = theta_right;

        // Do the PID control
        updateMotorsPID();

        // Publish the encoder values
        left_encoder_msg.data = theta_left;
        right_encoder_msg.data = theta_right;
        left_encoder_pub.publish(&left_encoder_msg);
        right_encoder_pub.publish(&right_encoder_msg);

        previousMillis = currentMillis;
    }

    nh.spinOnce();
}

// PID control function
void updateMotorsPID(){
    // Turn pwm values to expected speed
    float target_speed_left = target_pwm_left * 0.1;
    float target_speed_right = target_pwm_right * 0.1;

    // Calculate the error
    // The input pwm turn to speed - the speed from encoder
    float error_left = target_speed_left - speed_left_filtered;
    float error_right = target_speed_right - speed_right_filtered;

    // Calculate the integral
    integral_left += error_left * interval / 1000; // Convert to seconds
    integral_right += error_right * interval / 1000; // Convert to seconds

    //Constraint the integral to prevent windup
    integral_left = constrain(integral_left, -50, 50);
    integral_right = constrain(integral_right, -50, 50);

    // Calculate the derivative
    float derivative_left = (error_left - error_left_prev) / (interval / 1000); // Convert to seconds
    float derivative_right = (error_right - error_right_prev) / (interval / 1000); // Convert to seconds

    // Remain the previous error
    error_left_prev = error_left;
    error_right_prev = error_right;
    // Calculate the PID output
    float output_left = kp*error_left + ki*integral_left + kd*derivative_left;
    float output_right = kp*error_right + ki*integral_right + kd*derivative_right;

    // output_left and output_right means how much can we add to pwm to match the target pwm
    pwm_left = target_pwm_left + output_left;
    pwm_right = target_pwm_right + output_right;

    // Constrain the pwm values to be within the range of -255 to 255
    pwm_left = constrain(pwm_left, -255, 255);
    pwm_right = constrain(pwm_right, -255, 255);

    // Set the motor speed and direction
    setMotorPWM(pwm_left, pwm_right);
}

void setMotorPWM(int pwm_left, int pwm_right){
    // Set the motor speed and direction
    if(pwm_left >= 0){
        digitalWrite(AIN1, 1);
        digitalWrite(AIN2, 0);
        analogWrite(PWMA, pwm_left);
    }
    else{
        digitalWrite(AIN1, 0);
        digitalWrite(AIN2, 1);
        analogWrite(PWMA, -pwm_left);
    }
    if(pwm_right >= 0){
        digitalWrite(BIN1, 1);
        digitalWrite(BIN2, 0);
        analogWrite(PWMB, pwm_right);
    }
    else{
        digitalWrite(BIN1, 0);
        digitalWrite(BIN2, 1);
        analogWrite(PWMB, -pwm_right);
    }
}

void initMotor(){
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(PWMA, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(PWMB, OUTPUT);
    pinMode(STBY, OUTPUT);

    digitalWrite(STBY, HIGH); // Enable the motor driver
}
void initEncoder(){
    // Set the encoder A to input_pullup
    // Set encoder B to input
    // When A from kow to high it call the doEncoder_right function to interrupt
    pinMode(Pin_Encoder_Right_A, INPUT_PULLUP);
    pinMode(Pin_Encoder_Right_B, INPUT);
    attachInterrupt(digitalPinToInterrupt(Pin_Encoder_Right_A), doEncoder_right, RISING);

    pinMode(Pin_Encoder_Left_A, INPUT_PULLUP);
    pinMode(Pin_Encoder_Left_B, INPUT);
    attachInterrupt(digitalPinToInterrupt(Pin_Encoder_Left_A), doEncoder_left, RISING);
}

void doEncoder_right(){
    if(digitalRead(Pin_Encoder_Right_B) == HIGH){
        theta_right++;
    }
    else{
        theta_right--;
    }
}

void doEncoder_left(){
    if(digitalRead(Pin_Encoder_Left_B) == HIGH){
        theta_left++;
    }
    else{
        theta_left--;
    }
}
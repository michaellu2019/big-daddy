#include <PS2X_lib.h>
#include <Servo.h>
#include "Wire.h"
#include <MPU6050_light.h>
#include <ezButton.h>

// game states
enum Direction {LEFT, RIGHT};
enum AutonomousState {NOT_STARTED, FIRST_TURN, DRIVE_STRAIGHT, SECOND_TURN, LINE_FOLLOWING, FIRST_LIFT, LIFT_RESET, SECOND_LIFT, DONE};

// MPU6050 IMU settings
MPU6050 mpu(Wire);
unsigned long mpu_timer = 0;
double mpu_yaw = 0.0;

// photoresistor settings
#define PHOTORESISTOR_PIN A0
const int photoresistor_threshold = 40;
int photoresistor_value = 0;
bool autonomous_run;
int autonomous_start_time = -1;
int autonomous_drive_straight_start_time = -1;
int autonomous_first_lift_start_time = -1;
int autonomous_lift_reset_start_time = -1;
const float autonomous_left_servo_bal = 1.0465;
const int autonomous_first_turn_amount = 5;
const int autonomous_second_turn_amount = 75;
const int autonomous_drive_straight_duration = 2500;
const int autonomous_first_lift_duration = 9000;
const int autonomous_lift_reset_duration = 3000;
AutonomousState autonomous_state = NOT_STARTED;
Direction autonomous_turn_direction = RIGHT;
bool use_autonomous_line_follower = true;

// line follower settings
#define LEFT_LINE_FOLLOWER_PIN A1
#define RIGHT_LINE_FOLLOWER_PIN A2
const int line_follower_white_threshold = 200;
const int line_follower_black_threshold = 700;
int left_line_follower_value = 0;
int right_line_follower_value = 0;

// limit switch settings
#define LIFT_TOP_LIMIT_SWITCH_PIN 6
#define LIFT_BOTTOM_LIMIT_SWITCH_PIN 7
#define FRONT_LEFT_LIMIT_SWITCH_PIN 8
#define FRONT_RIGHT_LIMIT_SWITCH_PIN 9
const int limit_switch_debounce_time = 100;
ezButton lift_top_limit_switch(LIFT_TOP_LIMIT_SWITCH_PIN);
ezButton lift_bottom_limit_switch(LIFT_BOTTOM_LIMIT_SWITCH_PIN);
ezButton front_left_limit_switch(FRONT_LEFT_LIMIT_SWITCH_PIN);
ezButton front_right_limit_switch(FRONT_RIGHT_LIMIT_SWITCH_PIN);

// PS2 controller settings
#define PS2_DAT        2
#define PS2_CMD        3
#define PS2_ATT        4
#define PS2_CLK        5

#define PS2_ENABLE_PRESSURE    false
#define PS2_ENABLE_VIBRATE     false

#define PS2_JOYSTICK_NEUTRAL_VALUE 127
#define PS2_JOYSTICK_NEUTRAL_RANGE 4

PS2X ps2x;

int ps2_error = 0;
byte ps2_type = 0;  
byte ps2_vibrate = 0;

int ps2_left_joystick_x = 0;
int ps2_left_joystick_y = 0;
int ps2_right_joystick_x = 0;
int ps2_right_joystick_y = 0;

// servo settings
#define LEFT_SERVO_PIN 10
#define RIGHT_SERVO_PIN 11
#define LIFT_SERVO_PIN 12

// servo IDs
#define LEFT_SERVO_ID 1
#define RIGHT_SERVO_ID -1
#define LIFT_SERVO_ID 2

Servo left_servo;
Servo right_servo;
Servo lift_servo;

// servo speed tuning coefficients
const float left_servo_bal = 1.0;
const float right_servo_bal = 1.0;
const float lift_servo_bal = 1.0;

// servo speed values
#define MAX_DRIVE_SERVO_SPEED 1000
#define MIN_DRIVE_SERVO_SPEED 200
#define MAX_LIFT_SERVO_SPEED 700
const int neutral = 1500;
//const int forward = neutral + servoSpeed;
//const int backward = neutral - servoSpeed;
int left_servo_speed = 0;
int right_servo_speed = 0;
int lift_servo_speed = 0;
const float low_gear_bal = 0.2;
const float low_gear_left_bal = 1.1;
const float autonomous_forward_bal = 0.2;
const float autonomous_turn_bal = 0.35;
bool stowing_lift = false;

bool driving_servos = false;

// PID controller settings
unsigned long controller_timer;
double loop_time = 0.01;

struct PIDController {
  double Kp;
  double Ki;
  double Kd;
  double setpoint;
  double value;
  double error;
  double old_error;
  double d_error;
  double sum_error;
  double loop_time;
  double output;
};

PIDController turning_controller = { 5.0, 0.2, 0.1 };
double last_turning_setpoint = 0.0;
const double turning_setpoint_tolerance = 3.0;
const int turning_cooldown = 2000;
bool turning_setpoint_reached = false;

//const double[] turning_controller = {0.0, 0.0, 0.0};
//double turning_setpoint = 0.0;
//double turning_error = 0.0;
//double old_turning_error = 0.0;
//double sum_turning_error = 0.0;

// initializing function run once
void setup() {
  // configure Serial
  Serial.begin(115200);

  // LEDs
  pinMode(LED_BUILTIN, OUTPUT);
      
  // configure IMU
  Wire.begin();
  
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ }
  
  Serial.println(F("Calculating offsets for MPU6050, do not move!"));
  delay(1000);
  
  // mpu.upsideDownMounting = true;
  mpu.calcOffsets(); // gyro and accelero
  Serial.println("Done!\n");

  // configure photoresistor
  pinMode(PHOTORESISTOR_PIN, INPUT);
  autonomous_run = false;
  autonomous_start_time = -1;

  // configure line follower
  pinMode(LEFT_LINE_FOLLOWER_PIN, INPUT);
  pinMode(RIGHT_LINE_FOLLOWER_PIN, INPUT);

  // configure limit switches
  lift_top_limit_switch.setDebounceTime(limit_switch_debounce_time);
  lift_bottom_limit_switch.setDebounceTime(limit_switch_debounce_time);
  front_left_limit_switch.setDebounceTime(limit_switch_debounce_time);
  front_right_limit_switch.setDebounceTime(limit_switch_debounce_time);
  
  // configure ps2
  delay(300);
  configure_ps2();
  
  // configure servos
  pinMode(LEFT_SERVO_PIN, OUTPUT);
  pinMode(RIGHT_SERVO_PIN, OUTPUT);
  pinMode(LIFT_SERVO_PIN, OUTPUT);
  
  left_servo.attach(LEFT_SERVO_PIN);
  right_servo.attach(RIGHT_SERVO_PIN);
  lift_servo.attach(LIFT_SERVO_PIN);
  
  left_servo.writeMicroseconds(neutral);
  right_servo.writeMicroseconds(neutral);
  lift_servo.writeMicroseconds(neutral);
}

// loop function LINE_FOLLOWING forever
void loop() {
  mpu.update();

  lift_top_limit_switch.loop();
  lift_bottom_limit_switch.loop();
  front_left_limit_switch.loop();
  front_right_limit_switch.loop();
  
  if(millis() - mpu_timer > 10){
//    Serial.print("X : ");
//    Serial.print(mpu.getAngleX());
//    Serial.print("\tY : ");
//    Serial.print(mpu.getAngleY());
//    Serial.print("\tZ : ");
//    Serial.println(mpu.getAngleZ());
//    mpu_yaw = (int) mpu.getAngleZ() % 360 + (mpu.getAngleZ() - (int) mpu.getAngleZ());
//    mpu_yaw = mpu_yaw < 0.0 ? mpu_yaw + 360.0 : mpu_yaw;
    mpu_yaw = mpu.getAngleZ();
//    Serial.print("Z: ");
//    Serial.println(mpu_yaw);
    mpu_timer = millis();
  }
  
  resetServoSpeeds();

  left_line_follower_value = analogRead(LEFT_LINE_FOLLOWER_PIN);
  right_line_follower_value = analogRead(RIGHT_LINE_FOLLOWER_PIN);

//  if (autonomous_state != FIRST_LIFT)
//    autonomous_first_lift_start_time = millis();
//  if (autonomous_state != LIFT_RESET)
//    autonomous_lift_reset_start_time = millis();

  if (autonomous_state == NOT_STARTED) {
    photoresistor_value = analogRead(PHOTORESISTOR_PIN);
    Serial.println(photoresistor_value);
    if (photoresistor_value < photoresistor_threshold) {
      Serial.println("Run Autonomous!");
      autonomous_drive_straight_start_time = millis();
      autonomous_state = DRIVE_STRAIGHT;
//      autonomous_start_time = millis();
    }
  } else if (autonomous_state == FIRST_TURN) {
    turn_to_angle(((int) (turning_controller.value/90.0)) * 90.0 + autonomous_second_turn_amount * (autonomous_turn_direction == LEFT ? 1 : -1));
    if (turning_setpoint_reached) {
      autonomous_state = DRIVE_STRAIGHT;
    }
  } else if (autonomous_state == DRIVE_STRAIGHT) {
    left_servo_speed = MAX_DRIVE_SERVO_SPEED * autonomous_forward_bal * autonomous_left_servo_bal;
    right_servo_speed = MAX_DRIVE_SERVO_SPEED * autonomous_forward_bal;
    driveServo(LEFT_SERVO_ID, left_servo_speed);
    driveServo(RIGHT_SERVO_ID, right_servo_speed);
    
//    if (millis() - autonomous_drive_straight_start_time > autonomous_drive_straight_duration) {
//      autonomous_state = LINE_FOLLOWING;
//    }
//    Serial.print(left_line_follower_value);
//    Serial.print(" + ");
//    Serial.print(right_line_follower_value);
//    Serial.println();
    if (millis() - autonomous_drive_straight_start_time > autonomous_drive_straight_duration && 
        ((autonomous_turn_direction == LEFT && left_line_follower_value > line_follower_black_threshold) || (autonomous_turn_direction == RIGHT && right_line_follower_value > line_follower_black_threshold))) {
      autonomous_state = SECOND_TURN;
      Serial.println("GOT BLACK");
    }
  } else if (autonomous_state == SECOND_TURN) {
    turn_to_angle(((int) (turning_controller.value/90.0)) * 90.0 + autonomous_second_turn_amount * (autonomous_turn_direction == LEFT ? 1 : -1));
    if (turning_setpoint_reached) {
      autonomous_state = LINE_FOLLOWING;
    }
  } else if (autonomous_state == LINE_FOLLOWING) {
    Serial.print(left_line_follower_value);
    Serial.print(" - ");
    Serial.print(right_line_follower_value);
    Serial.println();
    if (left_line_follower_value < line_follower_black_threshold && right_line_follower_value > line_follower_white_threshold) {
      left_servo_speed = MAX_DRIVE_SERVO_SPEED * autonomous_forward_bal * (1 + autonomous_turn_bal);
      right_servo_speed = MAX_DRIVE_SERVO_SPEED * autonomous_forward_bal * 0.0;
      Serial.println("GO LEFT");
    } else if (left_line_follower_value > line_follower_white_threshold && right_line_follower_value < line_follower_black_threshold) {
      left_servo_speed = MAX_DRIVE_SERVO_SPEED * autonomous_forward_bal * 0.0;
      right_servo_speed = MAX_DRIVE_SERVO_SPEED * autonomous_forward_bal * (1 + autonomous_turn_bal);
      Serial.println("GO RIGHT");
    } else{
      left_servo_speed = MAX_DRIVE_SERVO_SPEED * autonomous_forward_bal;
      right_servo_speed = MAX_DRIVE_SERVO_SPEED * autonomous_forward_bal;
      Serial.println("GO STRAIGHT");
    }

    if (front_left_limit_switch.getState() == LOW && front_right_limit_switch.getState() == LOW) {
      resetServoSpeeds();
      autonomous_state = FIRST_LIFT;  
      autonomous_first_lift_start_time = millis();
    } else if (front_left_limit_switch.getState() == LOW && front_right_limit_switch.getState() == HIGH) {
      left_servo_speed = MAX_DRIVE_SERVO_SPEED * autonomous_forward_bal * 0.0;
      right_servo_speed = MAX_DRIVE_SERVO_SPEED * autonomous_forward_bal * (1 + autonomous_turn_bal);
    } else if (front_left_limit_switch.getState() == HIGH && front_right_limit_switch.getState() == LOW) {
      left_servo_speed = MAX_DRIVE_SERVO_SPEED * autonomous_forward_bal * (1 + autonomous_turn_bal);
      right_servo_speed = MAX_DRIVE_SERVO_SPEED * autonomous_forward_bal * 0.0;
    }
    
    driveServo(LEFT_SERVO_ID, left_servo_speed);
    driveServo(RIGHT_SERVO_ID, right_servo_speed);
  } else if (autonomous_state == FIRST_LIFT) {
    Serial.println("FIRST LIFT");
    if (lift_top_limit_switch.getState() == HIGH) {
      lift_servo_speed = MAX_LIFT_SERVO_SPEED;
      driveServo(LIFT_SERVO_ID, lift_servo_speed);
    }

    if (millis() - autonomous_first_lift_start_time > autonomous_first_lift_duration) {
//      autonomous_lift_reset_start_time = time_now;
      autonomous_state = LIFT_RESET;
    }
  } else if (autonomous_state == LIFT_RESET) {
    Serial.println("DOWN");
    if (lift_bottom_limit_switch.getState() == HIGH) {
      lift_servo_speed = -MAX_LIFT_SERVO_SPEED;  
      driveServo(LIFT_SERVO_ID, lift_servo_speed);
    }
    if (millis() - autonomous_first_lift_start_time > autonomous_first_lift_duration + autonomous_lift_reset_duration) {
      autonomous_state = SECOND_LIFT;
    }
  } else if (autonomous_state == SECOND_LIFT) {
    Serial.println("SECOND LIFT");
    if (lift_top_limit_switch.getState() == HIGH) {
      lift_servo_speed = MAX_LIFT_SERVO_SPEED;
    } else {
      autonomous_state = DONE;  
      Serial.println("DONE");
    }
    driveServo(LIFT_SERVO_ID, lift_servo_speed);
  }
  
//  if (millis() - autonomous_start_time > 30 * 1000) {
//    autonomous_state = DONE;
//  }
  
  if (ps2_error != 1 && ps2_type == 1) { // DualShock Controller
    teleop();
  } else {
    digitalWrite(LED_BUILTIN, LOW);
    Serial.println("Unrecognized controller?");
  } 
}

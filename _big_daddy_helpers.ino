void configure_ps2() {
  ps2_error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_ATT, PS2_DAT, PS2_ENABLE_PRESSURE, PS2_ENABLE_VIBRATE);

  Serial.print("Controller Error Status: ");
  Serial.print(ps2_error);
  Serial.println();
  
  if (ps2_error == 0){
    Serial.println("Found Controller, configured successfully.");
    
    ps2_type = ps2x.readType(); 
    switch (ps2_type) {
      case 0:
        Serial.print("Unknown Controller type found.\n");
        break;
      case 1:
        Serial.print("DualShock Controller found.\n");
        digitalWrite(LED_BUILTIN, HIGH);
        break;
      case 2:
        Serial.print("GuitarHero Controller found.\n");
        break;
    case 3:
        Serial.print("Wireless Sony DualShock Controller found.\n");
        break;
     }
     
    Serial.print("Pressure: ");
    Serial.print(PS2_ENABLE_PRESSURE);
    Serial.println();
    
    Serial.print("Vibrate: ");
    Serial.print(PS2_ENABLE_VIBRATE);
    Serial.println("\n");
  } else if (ps2_error == 1) {
    digitalWrite(LED_BUILTIN, LOW);
    Serial.println("No controller found, check wiring, see readme.txt to enable debug. visit www.billporter.info for troubleshooting tips...");
  } else if (ps2_error == 2) {
    digitalWrite(LED_BUILTIN, LOW);
    Serial.println("Controller found but not accepting commands. see readme.txt to enable debug. Visit www.billporter.info for troubleshooting tips...");
  } else if (ps2_error == 3) {
    digitalWrite(LED_BUILTIN, LOW);
    Serial.println("Controller refusing to enter Pressures mode, may not support it...");
  }
}

void update_controller(PIDController *controller) {
  controller->error = controller->setpoint - controller->value;
  controller->d_error = (controller->error - controller->old_error)/controller->loop_time;
  controller->old_error = controller->error;
  controller->sum_error = controller->sum_error + controller->error * loop_time;

  controller->output = controller->Kp * controller->error + controller->Ki * controller->sum_error + controller->Kd * controller->d_error;
}

void turn_to_angle(int setpoint) {
  turning_controller.value = mpu_yaw;
    turning_controller.setpoint = setpoint;
    if (abs(turning_controller.setpoint - turning_controller.value) < turning_setpoint_tolerance) {
      turning_controller.setpoint += 90.0 * (autonomous_turn_direction == LEFT ? 1 : -1);
      if (!turning_setpoint_reached) {
        turning_setpoint_reached = true;
        resetServoSpeeds();
        driveServo(LEFT_SERVO_ID, left_servo_speed);
        driveServo(RIGHT_SERVO_ID, right_servo_speed);
        Serial.println("REACHED SETPOINT");
      }
    } else {
      turning_setpoint_reached = false;
    }
    
    turning_controller.loop_time = loop_time;
    update_controller(&turning_controller);
    
    Serial.print("\Setpoint: ");
    Serial.print(turning_controller.setpoint);
    Serial.print("\tYaw: ");
    Serial.print(mpu_yaw);
    Serial.print("\tOutput: ");
    Serial.println(turning_controller.output);
      
//      turning_controller.output = max(MIN_DRIVE_SERVO_SPEED, abs(turning_controller.output)) * turning_controller.output/abs(turning_controller.output);
    turning_controller.output = (MIN_DRIVE_SERVO_SPEED + abs(turning_controller.output)) * turning_controller.output/abs(turning_controller.output);
    left_servo_speed = -turning_controller.output;
    right_servo_speed = turning_controller.output;
    driveServo(LEFT_SERVO_ID, left_servo_speed);
    driveServo(RIGHT_SERVO_ID, right_servo_speed);
}

void resetServoSpeeds() {
  left_servo_speed = 0;
  right_servo_speed = 0;
  lift_servo_speed = 0;
}

// function invoked to control servos
void driveServo(int servoId, int servoSpeed) {
//  Serial.print("Drive Servo ");
//  Serial.print(servoId);
//  Serial.print(" at speed ");
//  Serial.println(servoSpeed);
  if (servoId == LEFT_SERVO_ID) {
    left_servo.writeMicroseconds(neutral - left_servo_bal * servoSpeed);
  } else if (servoId == RIGHT_SERVO_ID) {
    right_servo.writeMicroseconds(neutral + right_servo_bal * servoSpeed);
  } else if (servoId == LIFT_SERVO_ID) {
    lift_servo.writeMicroseconds(neutral - lift_servo_bal * servoSpeed);  
  }
}

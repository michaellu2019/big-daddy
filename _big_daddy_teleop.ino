void teleop() {
  ps2x.read_gamepad(false, ps2_vibrate);
  digitalWrite(LED_BUILTIN, HIGH);
  driving_servos = false;
  
  if (ps2x.Button(PSB_START)) {
    Serial.println("Start pressed.");
  }
  if (ps2x.Button(PSB_SELECT)) {
    Serial.println("Select pressed."); 
    autonomous_state = DONE;
  }     
  if (ps2x.Button(PSB_PAD_UP)) {
    Serial.println("Up Pad pressed.");
    left_servo_speed = MAX_DRIVE_SERVO_SPEED * low_gear_bal * low_gear_left_bal;
    right_servo_speed = MAX_DRIVE_SERVO_SPEED * low_gear_bal;
  }
  if (ps2x.Button(PSB_PAD_DOWN)) {
    Serial.println("Down Pad pressed.");
    left_servo_speed = MAX_DRIVE_SERVO_SPEED * -low_gear_bal * low_gear_left_bal;
    right_servo_speed = MAX_DRIVE_SERVO_SPEED * -low_gear_bal;
  }
  if (ps2x.Button(PSB_PAD_LEFT)) {
    // && millis() - left_turning_cooldown > turning_cooldown) {
    Serial.println("Left Pad pressed.");
    left_servo_speed = MAX_DRIVE_SERVO_SPEED * -low_gear_bal * low_gear_left_bal;
    right_servo_speed = MAX_DRIVE_SERVO_SPEED * low_gear_bal;
//
//      turning_controller.value = mpu_yaw;
//      turning_controller.setpoint = ((int) (turning_controller.value/90.0)) * 90.0 + 90.0;
//      if (abs(turning_controller.setpoint - turning_controller.value) < turning_setpoint_tolerance) {
//        turning_controller.setpoint += 90.0;
//        if (!left_turning_setpoint_reached) {
//          left_turning_setpoint_reached = true;
//          left_turning_cooldown = millis();
//        }
//      } else {
//        left_turning_setpoint_reached = false;
//      }
//      
//      turning_controller.loop_time = loop_time;
//      update_controller(&turning_controller);
//      
//      Serial.print("\Setpoint: ");
//      Serial.print(turning_controller.setpoint);
//      Serial.print("\tYaw: ");
//      Serial.print(mpu_yaw);
//      Serial.print("\tOutput: ");
//      Serial.println(turning_controller.output);
//
//      left_servo_speed = (int) -turning_controller.output;
//      right_servo_speed = (int) turning_controller.output;
//      driving_servos = true;
  }
  if (ps2x.Button(PSB_PAD_RIGHT)) {
    Serial.println("Right Pad pressed.");
    left_servo_speed = MAX_DRIVE_SERVO_SPEED * low_gear_bal * low_gear_left_bal;
    right_servo_speed = MAX_DRIVE_SERVO_SPEED * -low_gear_bal;
  }

  ps2_vibrate = ps2x.Analog(PSAB_CROSS);
  if (ps2x.Button(PSB_L1)) {
    Serial.println("L1 pressed.");
    autonomous_state = DONE;
  }
  if (ps2x.Button(PSB_L2)) {
    Serial.println("L2 pressed.");
  }
  if (ps2x.Button(PSB_R1)) {
    Serial.println("R1 pressed.");
    stowing_lift = false;
    if (lift_top_limit_switch.getState() == HIGH) {
      lift_servo_speed = MAX_LIFT_SERVO_SPEED;
    }
  }
  if (ps2x.Button(PSB_R2)) {
    Serial.println("R2 pressed.");
    stowing_lift = false;
    if (lift_bottom_limit_switch.getState() == HIGH) {
      lift_servo_speed = -MAX_LIFT_SERVO_SPEED;
    }
  }    

  if (ps2x.Button(PSB_TRIANGLE)) {
    Serial.println("Triangle pressed.");
  }    
  if(ps2x.Button(PSB_CROSS)) {
    Serial.println("X pressed.");
    stowing_lift = true;
  } 
  if(ps2x.Button(PSB_SQUARE)) {
    Serial.println("Square pressed.");
  } 
  if(ps2x.Button(PSB_CIRCLE)) {
    Serial.println("Circle pressed.");
  }

  ps2_left_joystick_x = ps2x.Analog(PSS_LX);
  ps2_left_joystick_y = ps2x.Analog(PSS_LY);
  ps2_right_joystick_x = ps2x.Analog(PSS_RX);
  ps2_right_joystick_y = ps2x.Analog(PSS_RY);

  if (abs(ps2_left_joystick_x - PS2_JOYSTICK_NEUTRAL_VALUE) >  PS2_JOYSTICK_NEUTRAL_RANGE || abs(ps2_left_joystick_y - PS2_JOYSTICK_NEUTRAL_VALUE) >  PS2_JOYSTICK_NEUTRAL_RANGE ||
      abs(ps2_right_joystick_x - PS2_JOYSTICK_NEUTRAL_VALUE) >  PS2_JOYSTICK_NEUTRAL_RANGE || abs(ps2_right_joystick_y - PS2_JOYSTICK_NEUTRAL_VALUE) >  PS2_JOYSTICK_NEUTRAL_RANGE) {
    left_servo_speed = -map(ps2_left_joystick_y, 0, 255, 0, 2 * MAX_DRIVE_SERVO_SPEED) + MAX_DRIVE_SERVO_SPEED;
    right_servo_speed = -map(ps2_right_joystick_y, 0, 255, 0, 2 * MAX_DRIVE_SERVO_SPEED) + MAX_DRIVE_SERVO_SPEED;
//      driving_servos = true;
  
//      Serial.print("Left Joystick: ");
//      Serial.print(ps2_left_joystick_x);
//      Serial.print(", ");
//      Serial.print(ps2_left_joystick_y);
//      Serial.print(" --> ");
//      Serial.print(left_servo_speed);
//      Serial.print("    ");
//      Serial.print("Right Joystick: ");
//      Serial.print(ps2_right_joystick_x);
//      Serial.print(", ");
//      Serial.print(ps2_right_joystick_y);
//      Serial.print(" --> ");
//      Serial.print(right_servo_speed);
//      Serial.println();
  } 
  
//    if (!driving_servos) {
//      left_servo_speed = 0;
//      right_servo_speed = 0;
//    }

  if (stowing_lift && lift_bottom_limit_switch.getState() == HIGH) {
    lift_servo_speed = -MAX_LIFT_SERVO_SPEED;
  } else {
    stowing_lift = false;
  }
  
  driveServo(LEFT_SERVO_ID, left_servo_speed);
  driveServo(RIGHT_SERVO_ID, right_servo_speed);
  driveServo(LIFT_SERVO_ID, lift_servo_speed);
}

void power_motor(){
  // power up the motor
  digitalWrite(motorl1_enable, HIGH);
  digitalWrite(motorl2_enable, HIGH);
  digitalWrite(motorr1_enable, HIGH);
  digitalWrite(motorr2_enable, HIGH);
}

void set_motor_direction(int dir){
  // set up the motor direction pin to the direction you want
  // 1 = forward
  // 2 = backward
  // 3 = right
  // 4 = left
  switch(dir){
    case 2:
      digitalWrite(motorr1_dir, LOW);
      digitalWrite(motorr2_dir, LOW);
      digitalWrite(motorl1_dir, HIGH);
      digitalWrite(motorl2_dir, HIGH);
      break;
    case 1:
      digitalWrite(motorr1_dir, HIGH);
      digitalWrite(motorr2_dir, HIGH);
      digitalWrite(motorl1_dir, LOW);
      digitalWrite(motorl2_dir, LOW);
      break;
    case 3:
      digitalWrite(motorr1_dir, LOW);
      digitalWrite(motorr2_dir, LOW);
      digitalWrite(motorl1_dir, LOW);
      digitalWrite(motorl2_dir, LOW);
      break;
    case 4:
      digitalWrite(motorr1_dir, HIGH);
      digitalWrite(motorr2_dir, HIGH);
      digitalWrite(motorl1_dir, HIGH);
      digitalWrite(motorl2_dir, HIGH);
      break;
    default:
      break;
  }
}

void stop_motor(){
  // stop the motor, but not cut the power
  // becareful when you use this function, don't left the motor stop too fast!
  analogWrite(motorr1_pwm,0);
  analogWrite(motorr2_pwm,0);
  analogWrite(motorl1_pwm,0);
  analogWrite(motorl2_pwm,0); 
}

void cut_power(){
  // cut out power for all motors, helps to save the motor when stalling
  digitalWrite(motorl1_enable, LOW);
  digitalWrite(motorl2_enable, LOW);
  digitalWrite(motorr1_enable, LOW);
  digitalWrite(motorr2_enable, LOW);

//  digitalWrite(motorr1_dir, LOW);
//  digitalWrite(motorr2_dir, LOW);
//  digitalWrite(motorl1_dir, LOW);
//  digitalWrite(motorl2_dir, LOW);
  
  //stop_motor();
}

void report_error(){
  // stop the motor and report the error by turning on red LED
  // have to restart the Arduino to reset
  cut_power();
  blink_LED(1, 500);
  while(1){}
}

void turn_to_direction(float goal){
  // turn the robot to a certain degree
  // Assumption: 0 <= goal <= 360
  // return normally if success
  // light up red LED and lock itself if there is failure
  
  clear_compass_buffer();
  static int individual_speed[4] = {32,32,32,32};
  long last_Reading[4] = {0,0,0,0}; 
  long current_Reading[4] = {0,0,0,0};
  long target_speed = 2;
  float kp[4] = {3.0,3.0,3.0,3.0};
  int stall_counter[4] = {0,0,0,0};
  int max_stall_counter = 20;
  int max_stall_speed = 10;
  double accuracy = 0.3;
  int prev_direction = 1, current_direction = 1;

  leftEncoder.setEncoderCount(0);
  rightEncoder.setEncoderCount(0);
  leftEncoder2.setEncoderCount(0);
  rightEncoder2.setEncoderCount(0);
  unsigned long last_time = millis();
  long elapsed_time;
  float error = goal - get_direction_filter10();
  
  if (error > 180) error = error - 360;
  else if (error < -180) error = error + 360;

  power_motor();
  while (abs(error) > accuracy){
    // set up direction
    if (error > 0){
      set_motor_direction(3);
      current_direction = 1;
    }
    else{
      set_motor_direction(4);
      current_direction = 0;
    }
    
    // wait for a short time to avoid changing direction rapidly
    if (current_direction != prev_direction){
      cut_power();
      active_wait(500);
      power_motor();
      }
    prev_direction = current_direction;

    // get encoder reading
    current_Reading[0] = leftEncoder.getEncoderCount();
    current_Reading[1] = rightEncoder.getEncoderCount();
    current_Reading[2] = leftEncoder2.getEncoderCount();
    current_Reading[3] = rightEncoder2.getEncoderCount();
    elapsed_time = millis() - last_time;
    last_time = millis();

    // calculate amount of motor rotation for this loop
    for (int i = 0; i<4;i++){last_Reading[i] = abs(current_Reading[i] - last_Reading[i]);}

    // check if any motor is not rotating, stop the alrorithm if any motor get stalled for too long
    for (int i = 0; i<4;i++){
      if (last_Reading[i] <= max_stall_speed){
        stall_counter[i] ++;
        if (stall_counter[i] >= max_stall_counter){
          cut_power();
          report_error();
        }
      }
      else {stall_counter[i] = 0;}
    }

    // use proportional controller to controll the motor speed to follow the desired speed
    for (int i = 0; i<4;i++){
      individual_speed[i] += (target_speed*elapsed_time - last_Reading[i])*kp[i]/elapsed_time;     
      individual_speed[i] = max(0,individual_speed[i]);
      individual_speed[i] = min(max_power,individual_speed[i]);
    }
    
     // output information to computer
     Serial.print(target_speed*elapsed_time);
     Serial.print(",");
     for (int i = 0; i<4;i++){
      Serial.print(last_Reading[i]);
      Serial.print(",");
      }
     Serial.print(elapsed_time);
     Serial.print(",");
      for (int i = 0; i<4;i++){
      Serial.print(individual_speed[i]);
      Serial.print(",");
      }
     Serial.println(".");

    // oupdate last reading
    for (int i = 0; i<4;i++){last_Reading[i] = current_Reading[i];}

    // drive motor
    analogWrite(motorr1_pwm,individual_speed[2]);
    analogWrite(motorr2_pwm,individual_speed[0]);
    analogWrite(motorl1_pwm,individual_speed[3]);
    analogWrite(motorl2_pwm,individual_speed[1]);

    // update error
    error = goal - get_direction_filter10();
    if (error > 180) error = error - 360;
    else if (error < -180) error = error + 360;
    active_wait(10);
   }

   // stop motor
   stop_motor();
   cut_power();
   return;
}

void drive_to_destination(long goal,float dir){
  // drive the robot straight for a certain distance
  // goal = distance you want to travel in the form of encoder reading 
  // dir = direction, not implemented, you can put in any number
  // return normally if success
  // if there is failure, light up red LED and lock itself 

  float error;
  int stall_counter = 0;
  int max_stall_counter = 20;
  int max_stall_speed = 10;
  
  int fast_speed,slow_speed,motor_speedd = motor_speed;
  motor_speedd = max(0,motor_speedd);
  motor_speedd = min(max_power,motor_speedd);
  
  long currentLeftEncoderCount = 0;
  long currentRightEncoderCount = 0;
  long distance_traveled = 0;
  long last_distance = 0;
  leftEncoder.setEncoderCount(0);
  rightEncoder.setEncoderCount(0);

  if (goal >= 0){
    set_motor_direction(1);
  }
  else{
    set_motor_direction(2);
  }
  
  power_motor();
  analogWrite(motorr1_pwm,motor_speedd);
  analogWrite(motorr2_pwm,motor_speedd);
  analogWrite(motorl1_pwm,motor_speedd);
  analogWrite(motorl2_pwm,motor_speedd);

  goal = abs(goal);
  while(distance_traveled < goal){
  avoid_obstacle();
// ----------- code for direction correction, not tested ------------
//    error = dir - get_direction_filter10();
//    if (error > 180) error = error - 360;
//    else if (error < -180) error = error + 360;
//
//    speed_scale = abs(error)/5.0*0.5;
//    fast_speed = motor_speed * (1.0 + speed_scale);
//    fast_speed = min(200,fast_speed);
//    slow_speed = motor_speed * (1.0 - speed_scale);
//    slow_speed = max(16,slow_speed);
//
////    Serial.print(dir,3);
////    Serial.print(" , ");
////    Serial.print(error,3);
////    Serial.print(" , ");
////    Serial.print(speed_scale);
////    Serial.print(" , ");
////    Serial.print(slow_speed);
////    Serial.print(" , ");
////    Serial.println(fast_speed);
//    
//    if (error < 0){
//      analogWrite(motorr1_pwm,fast_speed);
//      analogWrite(motorr2_pwm,fast_speed);
//      analogWrite(motorl1_pwm,slow_speed);
//      analogWrite(motorl2_pwm,slow_speed);
//    }
//    else{
//      analogWrite(motorr1_pwm,slow_speed);
//      analogWrite(motorr2_pwm,slow_speed);
//      analogWrite(motorl1_pwm,fast_speed);
//      analogWrite(motorl2_pwm,fast_speed);
//    }
    
    currentLeftEncoderCount = leftEncoder.getEncoderCount();
    currentRightEncoderCount = rightEncoder.getEncoderCount();
    distance_traveled = abs((currentLeftEncoderCount+currentRightEncoderCount)/2);

    if ((distance_traveled - last_distance) <= max_stall_speed){
      stall_counter ++;
      if (stall_counter >= max_stall_counter){
        cut_power();
        report_error();
      }
    }
    else {stall_counter = 0;}
    last_distance = distance_traveled;
    active_wait(10);
  }
  Serial.println(distance_traveled);
  stop_motor();
  cut_power();
  return;
}

void turn_right(float degree){
  // make the car turn right by certain amount of degree
  // if you want the car to turn left, input a negative number
  // return normally if success
  // if there is failure, light up red LED and lock itself 
  
  if (degree > 180) degree = degree - 360;
  else if (degree < -180) degree = degree + 360;
  
  float current_degree = get_direction_filter10();
  float goal = get_direction_filter10()+ degree;
  
  if (goal > 180) goal = goal - 360;
  else if (goal < -180) goal = goal + 360;
  
  turn_to_direction(goal);
}

void turn_for_a_period(int dir, int time_in_ms){
  // make the car turn right or left by certain amount of time
  // if dir > 0, turn right
  // if dir <=0, turn left
  // Total driving duration = time_in_ms
  // return normally if success
  // if there is failure, light up red LED and lock itself 

  static int individual_speed[4] = {32,32,32,32};
  long last_Reading[4] = {0,0,0,0}; 
  long current_Reading[4] = {0,0,0,0};
  long target_speed = 2;
  float kp[4] = {3.0,3.0,3.0,3.0};
  int stall_counter[4] = {0,0,0,0};
  int max_stall_counter = 20;
  int max_stall_speed = 10;

  leftEncoder.setEncoderCount(0);
  rightEncoder.setEncoderCount(0);
  leftEncoder2.setEncoderCount(0);
  rightEncoder2.setEncoderCount(0);
  
  
  power_motor();
  // set up direction
  if (dir > 0){set_motor_direction(3);}
  else{set_motor_direction(4);}

  long elapsed_time;
  unsigned long last_time = millis();
  unsigned long start = millis();
  while(millis()-start < time_in_ms){
    // get encoder reading
    current_Reading[0] = leftEncoder.getEncoderCount();
    current_Reading[1] = rightEncoder.getEncoderCount();
    current_Reading[2] = leftEncoder2.getEncoderCount();
    current_Reading[3] = rightEncoder2.getEncoderCount();
    elapsed_time = millis() - last_time;
    last_time = millis();

    // calculate amount of motor rotation for this loop
    for (int i = 0; i<4;i++){last_Reading[i] = abs(current_Reading[i] - last_Reading[i]);}

    // check if any motor is not rotating, stop the alrorithm if any motor get stalled for too long
    for (int i = 0; i<4;i++){
      if (last_Reading[i] <= max_stall_speed){
        stall_counter[i] ++;
        if (stall_counter[i] >= max_stall_counter){
          cut_power();
          report_error();
        }
      }
      else {stall_counter[i] = 0;}
    }

    // use proportional controller to controll the motor speed to follow the desired speed
    for (int i = 0; i<4;i++){
      individual_speed[i] += (target_speed*elapsed_time - last_Reading[i])*kp[i]/elapsed_time;     
      individual_speed[i] = max(0,individual_speed[i]);
      individual_speed[i] = min(max_power,individual_speed[i]);
    }
    
     // output information to computer
     Serial.print(target_speed*elapsed_time);
     Serial.print(",");
     for (int i = 0; i<4;i++){
      Serial.print(last_Reading[i]);
      Serial.print(",");
      }
     Serial.print(elapsed_time);
     Serial.print(",");
      for (int i = 0; i<4;i++){
      Serial.print(individual_speed[i]);
      Serial.print(",");
      }
     Serial.println(".");

    // oupdate last reading
    for (int i = 0; i<4;i++){last_Reading[i] = current_Reading[i];}

    // drive motor
    analogWrite(motorr1_pwm,individual_speed[2]);
    analogWrite(motorr2_pwm,individual_speed[0]);
    analogWrite(motorl1_pwm,individual_speed[3]);
    analogWrite(motorl2_pwm,individual_speed[1]);

    active_wait(10);
   }

   // stop motor
   cut_power();
   return;
}

void drive_straight(int dir, int time_in_ms){
  // drive the motor straight for a certain amount of time
  // if dir > 0, drive forward
  // if dir <=0, drive backward
  // Total driving duration = time_in_ms
  // return normally if success
  // if there is failure, light up red LED and lock itself 

  unsigned long prev_time;
  int stall_counter = 0;
  int max_stall_counter = 20;
  int max_stall_speed = 10;
  int motor_speedd = motor_speed;
  motor_speedd = max(0,motor_speedd);
  motor_speedd = min(max_power,motor_speedd);

  long currentLeftEncoderCount = 0;
  long currentRightEncoderCount = 0;
  long distance_traveled = 0;
  long last_distance = 0;
  leftEncoder2.setEncoderCount(0);
  rightEncoder2.setEncoderCount(0);
  
  if (dir > 0){
    set_motor_direction(1);
  }
  else{
    set_motor_direction(2);
  }
  
  power_motor();
  analogWrite(motorr1_pwm,motor_speedd);
  analogWrite(motorr2_pwm,motor_speedd);
  analogWrite(motorl1_pwm,motor_speedd);
  analogWrite(motorl2_pwm,motor_speedd);

  prev_time = millis();
  while(millis()-prev_time < time_in_ms){
    currentLeftEncoderCount = leftEncoder2.getEncoderCount();
    currentRightEncoderCount = rightEncoder2.getEncoderCount();
    distance_traveled = abs((currentLeftEncoderCount+currentRightEncoderCount)/2);
    if ((distance_traveled - last_distance) <= max_stall_speed){
      stall_counter ++;
      if (stall_counter >= max_stall_counter){
        cut_power();
        report_error();
      }
    }
    else {stall_counter = 0;}
    last_distance = distance_traveled;
    active_wait(10);
  }
  
  cut_power();
  return;
}

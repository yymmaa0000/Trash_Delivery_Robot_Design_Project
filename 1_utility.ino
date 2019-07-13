void wait_for_buttom() {
  // wait for buttom to be pushed
  int pushed = HIGH;
  while (digitalRead(button_pin) == LOW || pushed) {
    if (digitalRead(button_pin) == LOW) {
      pushed = LOW;
    }
  }
  delay(1000);
}

void active_wait(int time_to_wait_in_ms) {
  // wait for a certain time without using delay()
  lastMilli = millis();
  while (millis() - lastMilli < time_to_wait_in_ms) { }
}

void blink_LED(int ID, int frequency_in_ms) {
  // blink a LED under certain frequency, runs forever
  // ID = 1: red_LED
  // ID = 2: green_LED
  // ID = 3: yellow_LED
  // ID = 4: white_LED
  // delay time = frequency_in_ms

  digitalWrite(yellow_LED, LOW);
  digitalWrite(white_LED, LOW);
  digitalWrite(green_LED, LOW);
  digitalWrite(red_LED, LOW);

  int current_LED = 0;
  switch (ID) {
    case 1:
      current_LED = red_LED;
      break;
    case 2:
      current_LED = green_LED;
      break;
    case 3:
      current_LED = yellow_LED;
      break;
    case 4:
      current_LED = white_LED;
      break;
    default:
      return;
      break;
  }
  while (1) {
    digitalWrite(current_LED, HIGH);
    active_wait(frequency_in_ms);
    digitalWrite(current_LED, LOW);
    active_wait(frequency_in_ms);
  }
}

void avoid_obstacle() {
  const int threshold_to_stop = 30;
  static bool obstacle_detected = false;
  static int number_exceed_limit = 0;
  static int L_Distance_Sum = 0;
  static int R_Distance_Sum = 0;
  static int L_Distance[3];
  static int R_Distance[3];
  static int index = 0;
  static bool array_full = false;
  static int LPF_leftDist = 0;
  static int LPF_rightDist = 0;
  // Ultrasonic Signal LPF
  L_distance = GetDistance(L_TrigPin, L_EchoPin);
  R_distance = GetDistance(R_TrigPin, R_EchoPin);
  //Serial.print(L_distance); Serial.print("  "); Serial.print(R_distance); Serial.println(number_exceed_limit);
  if (array_full) {
    L_Distance_Sum -= L_Distance[index];
    R_Distance_Sum -= R_Distance[index];
  }
  L_Distance[index] = L_distance;
  R_Distance[index] = R_distance;
  L_Distance_Sum += L_distance;
  R_Distance_Sum += R_distance;
  index++;
  if (index == 3) {
    array_full = true;
    index = 0;
  }
  // Only executes when LPF signal is ready
  if (array_full) {
    LPF_leftDist = L_Distance_Sum * 0.3333;
    LPF_rightDist = R_Distance_Sum * 0.3333;
    if (LPF_leftDist == 0 || LPF_leftDist == -0) {
      LPF_leftDist = 400;
    }
    if (LPF_rightDist == 0 || LPF_rightDist == -0) {
      LPF_rightDist = 400;
    }
    //Serial.print(LPF_leftDist); Serial.print("  "); Serial.println(LPF_rightDist);
    if (LPF_leftDist < threshold_to_stop || LPF_rightDist < threshold_to_stop) {  // Accumulates consecutive detection signals
      number_exceed_limit++;
    }
    else {
      number_exceed_limit = 0;
    }
    if (number_exceed_limit > 1) {  // detected if accumulation exceeds some number
      number_exceed_limit = 0;
      obstacle_detected = true;
    }

    while (obstacle_detected) {   // stop motor if detected
      cut_power();
      digitalWrite(yellow_LED, HIGH);
      digitalWrite(white_LED, LOW);
      digitalWrite(green_LED, LOW);
      digitalWrite(red_LED, LOW);
      L_distance = GetDistance(L_TrigPin, L_EchoPin);
      R_distance = GetDistance(R_TrigPin, R_EchoPin);
      //Serial.print(L_distance); Serial.print("  "); Serial.print(R_distance); Serial.println(number_exceed_limit);
      L_Distance_Sum -= L_Distance[index];
      R_Distance_Sum -= R_Distance[index];
      L_Distance[index] = L_distance;
      R_Distance[index] = R_distance;
      L_Distance_Sum += L_distance;
      R_Distance_Sum += R_distance;
      index++;
      if (index == 3) {
        index = 0;
      }
      LPF_leftDist = L_Distance_Sum * 0.3333;
      LPF_rightDist = R_Distance_Sum * 0.3333;
      if (LPF_leftDist == 0 || LPF_leftDist == -0) {
        LPF_leftDist = 400;
      }
      if (LPF_rightDist == 0 || LPF_rightDist == -0) {
        LPF_rightDist = 400;
      }
      //Serial.print(LPF_leftDist); Serial.print("  "); Serial.println(LPF_rightDist);
      if (LPF_leftDist > threshold_to_stop && LPF_rightDist > threshold_to_stop) {  //detect whether the obstacle is still there
        number_exceed_limit++;
      }
      else {
        number_exceed_limit = 0;
      }
      if (number_exceed_limit > 1)
      {
        number_exceed_limit = 0;
        obstacle_detected = false;
      }
      active_wait(200);
    }
  }
  active_wait(200);
  power_motor();
  digitalWrite(yellow_LED, LOW);
  return;
}

void wait_for_placement()
// for ultrasonic sensor
{
  while (1)
  {
    ultrasonic_distance = GetDistance(TrigPin, EchoPin);
    delay(200);
    if (ultrasonic_distance < 20) break;
  }
}

int GetDistance (int SR04_TrigPin, int SR04_EchoPin) {
  digitalWrite(SR04_TrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(SR04_TrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(SR04_TrigPin, LOW);
  int SR04_distance = pulseIn(SR04_EchoPin, HIGH) / 58.0;
  return SR04_distance;
}

void calibration(){
  leftEncoder.setEncoderCount(0);
  rightEncoder.setEncoderCount(0);
  target_number = 0;
  int current_state = 0; // 0 = turning state, 1 = driving state

  while(digitalRead(button_pin) == LOW){

    if(Serial.available()){
    BT_COM=Serial.read();
    //Serial.println("Activated!");
    if (BT_COM == 1)                    // turn left
              {
                if (current_state == 1){
                  target_dis[target_number] = (leftEncoder.getEncoderCount()+rightEncoder.getEncoderCount())/2;
                  current_state = 0;
                  target_number++;
              }
              turn_for_a_period(0,ms_time);        
             //Serial.println(1);
             }
       
    if (BT_COM == 2){                   // turn right
             if (current_state == 1){
                  target_dis[target_number] = (leftEncoder.getEncoderCount()+rightEncoder.getEncoderCount())/2;
                  current_state = 0;
                  target_number++;
              }
             turn_for_a_period(1,ms_time);  
             //Serial.println(2);
             }

    if (BT_COM == 3){                   // backward
             if (current_state == 0){
                  leftEncoder.setEncoderCount(0);
                  rightEncoder.setEncoderCount(0);
                  target_dir[target_number] = get_direction_filter10();
                  current_state = 1;
              }
              drive_straight(0, ms_time);
             //Serial.println(3);
             }

    if (BT_COM == 4){                   // forward
             if (current_state == 0){
                  leftEncoder.setEncoderCount(0);
                  rightEncoder.setEncoderCount(0);
                  target_dir[target_number] = get_direction_filter10();
                  current_state = 1;
              }
             drive_straight(1, ms_time);
             //Serial.println(4);
             }
    }
  }
  if (current_state == 1){
   target_dis[target_number] = (leftEncoder.getEncoderCount()+rightEncoder.getEncoderCount())/2;
   current_state = 0;
   target_number++;
  }
}

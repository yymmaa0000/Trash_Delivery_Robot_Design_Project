void go_to_curb(){
  for (int i = 0; i < target_number;i++){
    turn_to_direction(target_dir[i]);
    active_wait(100);
    drive_to_destination(target_dis[i],target_dir[i]);
    active_wait(1500);
  }
}

void go_home(){
  for (int i = target_number-1; i >= 0;i--){
    float new_dir = target_dir[i] +180;
    if (new_dir >= 360)new_dir -= 360;
    
    turn_to_direction(new_dir);
    active_wait(100);
    drive_to_destination(target_dis[i],new_dir);
    active_wait(1500);
  }
}

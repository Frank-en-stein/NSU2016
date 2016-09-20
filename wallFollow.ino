
void wallget() {
    mySerial.flush();
    bluef();
      reading = (char) mySerial.read();
      //Serial.print(reading);
      //Serial.print(" ");
      if(reading==125) {
        do{
          SM =(char) mySerial.read();
        }while(SM==-1);
        do{
          SL =(char) mySerial.read();
        }while(SL==-1);
        do{
          SR =(char) mySerial.read();
        }while(SR==-1);
      
    //Serial.println();
  }
}

void front_sonar() {
    mySerial.flush();
    bluef();
      reading = (char) mySerial.read();
      //Serial.print(reading);
      //Serial.print(" ");
      if(reading==125) {
        do{
          SM =(char) mySerial.read();
        }while(SM==-1);
      }
}

void wallfollowRight() {
  if(SR==0) {
    //stop();
    return;
  }
  int error = SR - setpoint_wall;
  if(SM>0 && SM<16){
    do{
      wheel(-90,90);
      wallget();
    }while(SM>0);
  }
  int turn = error*kp_wall + (error - lasterror_wall)*kd_wall;
  
  MLeft = cave_throttle + turn;
  MRight = cave_throttle - turn;
  if (MLeft > cave_MAX) MLeft = cave_MAX;
  if (MRight > cave_MAX) MRight = cave_MAX;
  if (MLeft < -cave_revMAX) MLeft = -cave_revMAX;		// to avoid extra reverse thrust //
  if (MRight < -cave_revMAX) MRight = -cave_revMAX;		// to avoid extra reverse thrust //
    
  lasterror_wall = error;
  wheel(MLeft, MRight);
}

void wallfollowLeft() {
  if(SL==0) {
    //stop();
    return;
  }
  int error = setpoint_wall - SL;
  if(SM>0 && SM<16){
    do{
      wheel(-90,90);
      wallget();
    }while(SM>0);
  }
  int turn = error*kp_wall + (error - lasterror_wall)*kd_wall;
  
  MLeft = cave_throttle + turn;
  MRight = cave_throttle - turn;
  if (MLeft > cave_MAX) MLeft = cave_MAX;
  if (MRight > cave_MAX) MRight = cave_MAX;
  if (MLeft < -cave_revMAX) MLeft = -cave_revMAX;		// to avoid extra reverse thrust //
  if (MRight < -cave_revMAX) MRight = -cave_revMAX;		// to avoid extra reverse thrust //
    
  lasterror_wall = error;
  wheel(MLeft, MRight);
}


void lineFollowing()
    {
      errorCalculation();
      PID();
      motorControl();
    }
    
    
    
void PID()
{
  if(currentError == 0) diffError = 0;
  else if(currentError == -7) return;
  else if(currentError != 0) diffError = currentError - previousError;
  pwm = (Kp*currentError) + (Kd*diffError) + (Ki*integral);
  previousError = currentError;
  integral+=currentError;
}


void motorControl()
{
  /*if(currentError == 0){
   		wheel(250, 250);     //will be made 250 if slow
   
   	}*/
  /*else if(currentError == -7) {
   		delay(relax);
   //wheel(speedL, speedR);
   		return;
   	}*/
  //else 
  {
    MLeft = speedL + (pwm<0 ? pwm:0);
    MRight = speedR - (pwm<0 ? 0:pwm);
//    MLeft = speedL+pwm;
//    MRight = speedR-pwm;
    if (MLeft > MAX) MLeft = MAX;
    if (MRight > MAX) MRight = MAX;
    if (MLeft < -revMAX) MLeft = -revMAX;		// to avoid extra reverse thrust //
    if (MRight < -revMAX) MRight = -revMAX;		// to avoid extra reverse thrust //
    wheel(MLeft, MRight);
  }
}

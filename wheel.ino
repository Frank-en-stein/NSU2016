void dash() {
  wheel(255, 255);
  delay(250);
  hardbreak();
  delay(10);
}


void wheel(int l, int r)
{ 
  if(l > 0) {
    digitalWrite(motorLp, HIGH);
    digitalWrite(motorLn, LOW);
  }
  else if(l < 0) {
    digitalWrite(motorLp, LOW);
    digitalWrite(motorLn, HIGH);
  }
  else {
    digitalWrite(motorLp, HIGH);
    digitalWrite(motorLn, HIGH);
  }
  if(r > 0) {
    digitalWrite(motorRp, HIGH);
    digitalWrite(motorRn, LOW);
  }
  else if(r < 0) {
    digitalWrite(motorRp, LOW);
    digitalWrite(motorRn, HIGH);
  }
  else {
    digitalWrite(motorRp, HIGH);
    digitalWrite(motorRn, HIGH);
  }
  
  previousSpeedL = l;
  previousSpeedR = r;

  if(l<0) l = abs(l);
  if(r<0) r = abs(r);

  analogWrite(motorR, r);
  analogWrite(motorL, l);
}

void wheel_alt(int m2Speed, int m1Speed) {
  analogWrite(motorR, abs(m1Speed));
  analogWrite(motorL, abs(m2Speed));
  digitalWrite(motorRp, (m1Speed >= 0 ? HIGH:LOW));
  digitalWrite(motorRn, (m1Speed < 0 ? HIGH:LOW));
  digitalWrite(motorLp, (m2Speed >= 0 ? HIGH:LOW));
  digitalWrite(motorLn, (m2Speed < 0 ? HIGH:LOW));
}


void discontinutyLeh()
{
  wheel(speedL, speedR);
  delay(15);
  // anything more?? // 
}
void bluef(){
  digitalWrite(blue, HIGH);
  delay(10);
  digitalWrite(blue, LOW);
}
void redf(){
  digitalWrite(red,HIGH);
  delay(10);
  digitalWrite(red,LOW);
}
void beepbuzzer()
{
  digitalWrite(buzzer, HIGH);
  delay(10);
  digitalWrite(buzzer , LOW);
}






void getInTrack()
{

  errorCalculation();

  if(currentError==0);

  else if(currentError>0)
  {
    while(currentError!=0)
    {
      errorCalculation();
      wheel(100, -100);
    }

    wheel(-100,100);
    delay(20);
    wheel(0,0);
//    delay(20);
  }

  else if(currentError<0)
  {
    while(currentError!=0)
    {
      errorCalculation();
      wheel(-100, 100);
    }
    wheel(100,-100);
    delay(20);
    wheel(0,0);
//    delay(20);
  }
}


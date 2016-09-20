void hardBreak()
{
  wheel(-200, -200);
  delay(20);
  wheel(0, 0);
  delay(10);
}

void o_hardBreak()
{
  wheel(-250, -250);
  delay(40);
  wheel(0,0);
  delay(10);
}


void stop()
{
//  wheel(-200, -200);
//  delay(50);
  wheel(0, 0);
  delay(50);		// this delay is important, applied by Khaled Bhai //
}

void forward()
{
  wheel(previousSpeedL, previousSpeedR);
}
void forward2()
{
  bluef();
  redf();
  beep();
  wheel(170,200);
  delay(3000);
  wheel(-200,-200);
  delay(20);
  wheel(0,0);
  delay(10);
  bluef();
  redf();
  beep();
}

void blindLeft90() {
  hardbreak();
  delay(10);
  wheel(-100, 100);
  delay(380);
  
  wheel(100, -100);
  delay(50);
  wheel(0,0);
  delay(50);
}

void blindRight90() {
  hardbreak();
  delay(10);
  wheel(100, -100);
  delay(380);
  
  wheel(-100, 100);
  delay(50);
  wheel(0,0);
  delay(50);
}

void intersection_RightTurn(){
//  wheel(200,200);
//  delay(80);
  stop();
  delay(10);
  while(!((analogRead(MR) > threshhold[6]) ? !blackline:blackline)){
    wheel(90,-90);
    
  }
  while(!((analogRead(C) > threshhold[3]) ? !blackline:blackline)){
    wheel(90,-90);
  }
  
  wheel(-255,255);
  delay(50);
  wheel(0,0);
  delay(15);
  
  //delay(5000);
}
void intersection_LeftTurn(){
//  wheel(200,200);
//  delay(80);
  stop();
  delay(10);
  while(!((analogRead(ML) > threshhold[0]) ? !blackline:blackline)){
    wheel(-90,90);
   
  }
  while(!((analogRead(C) > threshhold[3]) ? !blackline:blackline)){
    wheel(-90,90);
   
  }
  wheel(255,-255);
  delay(50);
  wheel(0,0);
  delay(15);
  
  //delay(5000);
}


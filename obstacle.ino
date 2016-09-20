void go_around() {
  errorCalculation();
  if((int)currentError%2==1) {
    wheel(-190, 0);
    do{
      errorCalculation();
    }while((int)currentError%2==1);
    wheel(0,0);
    delay(10);  
  }
  
  blindLeft90();
  dash();
  blindRight90();
  int time=30;
  wheel(90, 90);
  do{
    wallget();
    delay(1);
    time++;
  }while(SR==0);
  hardbreak();
  delay(10);
  do{
    wallget();
    wallfollowRight();
  }while(SR>0);
  hardbreak();
  delay(10);
  wheel(90,90);
  while(time--) {
    wallget();
    delay(1);
  }
  blindRight90();
  wheel(90, 90);
  int line;
  do{
    line = irUpdate();
  }while(line==0);
  do{
    line = irUpdate();
  }while(line!=0);
  hardbreak();
  delay(30);
  wheel(0, 120);
  do{
    irUpdate();
  }while(S[3]==0);
  lineFollowing();
}
/*void obstacle()
{
  if(mySerial.available()){
       x = mySerial.read();
//      if(x == 'n'){
//        //Serial.println("Forward!!!");
//       
//      } 
      if(x == 'o'){
        wheel(-100, -100);
        delay(60);
        obstacle_turn_l();
        mySerial.flush();
        x = '1';
      }
     x = '1'; //Serial.println("TURN!!!"); 
   }
}*/

/*void obstacle_turn_l()
{
  o_hardBreak();
  beep();
//  while(1){
//    
//  }
  wheel(-180, 180);
  delay(140);
  //wheel(-20,-20);
   //delay(20);
  wheel(120,120);
  delay(25);
   beep();  
//beep();
  //wheel(120,200);
  //delay(40);                                         //beep();
 beep();
 
 /* wheel(180,35);
  delay(950);
  beep();
   wheel(-20,-20);
   delay(20);
   wheel(0,0);
   delay(800);
                                         //beep();
  wheel(150,10);
  delay(100);
  beep();
   wheel(-20,-20);
   delay(20);
   wheel(0,0);
   delay(800);*/
                                            //beep();
 /* errorCalculation();
  while(!(analogRead(C)>threshhold[4] || analogRead(CR)>threshhold[5])) {
    
    wheel(180,35);
    errorCalculation();
//    while(1){
//      wheel(0,0);
//    }
  }
  beep();
  wheel(140, 140);
  delay(140);

  o_leftTurn();
  getInTrack();
  x = '1';
}*/

/*void obstacle_turn_r()
{
  
  o_hardBreak();
  beep();
//  while(1){
//    
//  }
  wheel(180, -180);
  delay(140);
   beep();                                       //beep();
  wheel(200,120);
  delay(40);                                         //beep();
 beep();
 
  wheel(30,130);
  delay(800);
  beep();
                                         //beep();
  wheel(30,105);
  delay(550);
  beep();
                                            //beep();
  errorCalculation();
  while(!(analogRead(CL)>threshhold[2] || analogRead(C)>threshhold[3] || analogRead(CR)>threshhold[4])) {
    
    wheel(200,200);
    errorCalculation();
//    while(1){
//      wheel(0,0);
//    }
  }
  beep();
  wheel(140, 140);
  delay(140);

  o_rightTurn();
  getInTrack();
  x = '1';
  
}*/


void beep()
{
  digitalWrite(buzzer, HIGH);
  delay(10);
  digitalWrite(buzzer, LOW);
//  delay(10);
//  digitalWrite(buzzer, HIGH);
//  delay(10);
//  digitalWrite(buzzer, LOW);
}


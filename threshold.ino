


void threshholdread()
{
  wheel(calibrationSpeed, -calibrationSpeed);
  int i; 
  for(i = 0; i < 5000; i++) {
    ir0 = analogRead(ML);
    ir1 = analogRead(L);
    ir2 = analogRead(CL);
    ir3 = analogRead(C);
    ir4 = analogRead(CR);
    ir5 = analogRead(R);
    ir6 = analogRead(MR);

    if(ir0<white[0]) white[0]  = ir0;
    if(ir1<white[1]) white[1]  = ir1;
    if(ir2<white[2]) white[2]  = ir2;
    if(ir3<white[3]) white[3]  = ir3;
    if(ir4<white[4]) white[4]  = ir4;
    if(ir5<white[5]) white[5]  = ir5;
    if(ir6<white[6]) white[6]  = ir6;

    if(ir0>black[0]) black[0]  = ir0;
    if(ir1>black[1]) black[1]  = ir1;
    if(ir2>black[2]) black[2]  = ir2;
    if(ir3>black[3]) black[3]  = ir3;
    if(ir4>black[4]) black[4]  = ir4;
    if(ir5>black[5]) black[5]  = ir5;
    if(ir6>black[6]) black[6]  = ir6;
  }
  stop();
  threshhold[0] = (white[0] + black[0])*0.65;
  threshhold[1] = (white[1] + black[1])*0.65;
  threshhold[2] = (white[2] + black[2])*0.65;
  threshhold[3] = (white[3] + black[3])*0.65;
  threshhold[4] = (white[4] + black[4])*0.65;
  threshhold[5] = (white[5] + black[5])*0.65;
  threshhold[6] = (white[6] + black[6])*0.65;

  for(int i = 0; i < 7; i++){
    int a = map(threshhold[i],0,1023,0,255);
    EEPROM.write(i, a);
  }
}




void threshholdget()
{
  for(int i = 0; i < 7; i++) {
    threshhold[i] = map(EEPROM.read(i),0,255,0,1023);
  }
}

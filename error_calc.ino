void errorCalculation()
{
  s[0]=analogRead(ML);
  s[1]=analogRead(L);
  s[2]=analogRead(CL);
  s[3]=analogRead(C);
  s[4]=analogRead(CR);
  s[5]=analogRead(R);
  s[6]=analogRead(MR);

  int avg = 0, val = 0;
  sum=0;
  for (int i = 0; i < 7; i++)
  {
    S[i] = (s[i] > threshhold[i]) ? !blackline : blackline;
//    Serial.print(s[i]);
//    Serial.print(' ');
    avg = avg + (S[i] * i);
    sum = sum + S[i];
  }
  if(sum == 0) val = -1;
  else val = int((avg * 2) / sum);
  currentError = val - 6;      //careful what u wish for :p
}



int irUpdate()
{
  s[0]=analogRead(ML);
  s[1]=analogRead(L);
  s[2]=analogRead(CL);
  s[3]=analogRead(C);
  s[4]=analogRead(CR);
  s[5]=analogRead(R);
  s[6]=analogRead(MR);
  int sum=0;
  for (int i = 0; i < 7; i++)
  {
    S[i] = (s[i] > threshhold[i]) ? !blackline:blackline;
    sum+=S[i];
  }
  return sum;
}

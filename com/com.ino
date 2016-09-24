#include <SoftwareSerial.h>
#include <NewPing.h>

#define trig1 4
#define echo1 5
#define max_distance 35
#define obstacle_distance 50
#define trig2 8
#define echo2 9
#define trig3 6
#define echo3 7

NewPing sonarL(trig3, echo3, max_distance);
NewPing sonarR(trig1, echo1, max_distance);
NewPing sonarM(trig2, echo2, obstacle_distance);

SoftwareSerial mySerial(2, 3); // RX, TX


void setup() {
  Serial.begin(9600);
  mySerial.begin(9600);
  //mySerial.println("Hello, world");
}

void loop() { // run over and over
//mySerial.write(125);//delay(10);
//mySerial.write(101);//delay(10);
//mySerial.write(101);delay(30);return;
  
  unsigned int SL = sonarL.ping_cm();
  unsigned int SR = sonarR.ping_cm();
  unsigned int SM = sonarM.ping_cm();
  
  
  mySerial.write(125);
  mySerial.write(SM);
  mySerial.write(SL);
  mySerial.write(SR);
  delay(10);
  
 Serial.print(SL);
 Serial.print(' ');
 Serial.print(SM);
 Serial.print(' ');
 Serial.println(SR);
 
}

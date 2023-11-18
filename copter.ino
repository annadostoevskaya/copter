/**
   File: copter.ino
   Author: github.com/annadostoevskaya
   Date: 11/10/2023 20:13:34
   Last Modified Date: 11/10/2023 20:13:43
   
   Bluetooth:
    - Serial1
    - 19 - TXD (for arduino RX)
    - 18 - RXD (for arduino TX)
*/ 

#include <Servo.h>

#define BTSerial Serial1 // 19, 18

Servo motors[4];

void setup()
{
  Serial.begin(9600);
  BTSerial.begin(9600);
  /*
  int shift = 2;
  int port = 0;
  for (int i = 0; i < 4; i += 1)
  {
    port = shift + i;
    motors[i].attach(port);
  }

  for (int i = 0; i < 4; i += 1)
    motors[i].writeMicroseconds(2300);

  delay(2000);

  for (int i = 0; i < 4; i += 1)
    motors[i].writeMicroseconds(800);

  delay(6000);
  */
}

void loop()
{
  ////////////////////////////////////////////////////////// 
  // int stick = map(analogRead(0), 0, 1023, 800, 2300);
  // for (int i = 0; i < 4; i += 1)
  //   motors[i].writeMicroseconds(stick);
  
  ////////////////////////////////////////////////////////// 
  int output = map(BTSerial.parseInt(), 0, 100, 800, 2300);
  Serial.println(output);
}

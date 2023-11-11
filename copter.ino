/**
   File: copter.ino
   Author: github.com/annadostoevskaya
   Date: 11/10/2023 20:13:34
   Last Modified Date: 11/10/2023 20:13:43
   
   41 - Vcc
   43 - GND
   45 - TX (for arduino RX)
   47 - RX (for arduino TX)
*/ 

#include <Servo.h>
#include <SoftwareSerial.h>

SoftwareSerial bluetooth(45, 47);

Servo motors[4];

void setup()
{
  Serial.begin(115200);
  bluetooth.begin(9600);
  
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

  // Bluetooth Module Init
  pinMode(41, OUTPUT);
  digitalWrite(41, HIGH);
  pinMode(43, OUTPUT);
  digitalWrite(43, LOW);
}

void loop()
{
  ////////////////////////////////////////////////////////// 
  // int stick = map(analogRead(0), 0, 1023, 800, 2300);
  // for (int i = 0; i < 4; i += 1)
  //   motors[i].writeMicroseconds(stick);

  ////////////////////////////////////////////////////////// 
  bluetooth.println("You look lonely, I can fix that...");
}

/**
    File: copter.ino
    Author: github.com/annadostoevskaya
    Date: 11/10/2023 20:13:34
    Last Modified Date: 11/20/2023 22:00:46
    
    Description:
        copter.ino is simple homebrew project for custom copter.
        TODO(annad): I add link on video in this file later.

    Bluetooth:
        TODO(annad): Write bluetooth library with support HardwareSerial.
        ---
        Serial1:
            * 19 - TXD (for arduino RX)
            * 18 - RXD (for arduino TX)

    Motors:
        * pins 2, 3, 4 & 5
*/ 

#include <Servo.h>

#define BTSerial Serial1 // 19, 18

Servo motors[4];

void setup()
{
  Serial.begin(9600);
  Serial.print(">");
  BTSerial.begin(9600);

  Serial.println("Serial: Initialized...");
  
  int shift = 2;
  int port = 0;
  for (int i = 0; i < 4; i += 1)
  {
    Serial.println("Servo: attach port");
    port = shift + i;
    motors[i].attach(port);
  }

  for (int i = 0; i < 4; i += 1)
  {
    Serial.println("Servo: setup, part 1");
    motors[i].writeMicroseconds(2300);
  }

  delay(2000);

  for (int i = 0; i < 4; i += 1)
  {
    Serial.println("Servo: setup, part 2");
    motors[i].writeMicroseconds(800);
    
  }
  delay(6000);
}

static int gas = 0;

void loop()
{
  int recived = BTSerial.parseInt();
  
  if (recived <= 100)
  {
    gas = map(recived, 0, 100, 800, 2300);
  }
  
  for (int i = 0; i < 4; i += 1)
    motors[i].writeMicroseconds(gas);
}

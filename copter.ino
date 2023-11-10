/**
 * File: copter.ino
 * Author: github.com/annadostoevskaya
 * Date: 11/10/2023 20:13:34
 * Last Modified Date: 11/10/2023 20:13:43
 */

#include <Servo.h>

Servo motors[4];

void setup() 
{
  Serial.begin(115200);

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
}

void loop() 
{
  int stick = map(analogRead(0), 0, 1023, 800, 2300);
  for (int i = 0; i < 4; i += 1)
    motors[i].writeMicroseconds(stick);
}

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

    Gyroscope:
        * D21 - SCL
        * D20 - SDA
*/ 

#include <Servo.h>
#include <stdint.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define BTSerial Serial1 // 19, 18

struct Euler
{
  float pitch, // nose up, or tail down (yes)
        yaw, // nose move from side to side (no)
        roll; // a circular (clockwise or anticlockwise)
  // movement of the body as it moves forward (maybe)
};

struct PID
{
  double p;
  double i;
  double d;

  double prev_err;
  double integral;
};

static Euler g_gyro_pos = {};
static Euler g_gyro_zero = {};
static Adafruit_MPU6050 mpu;
static Servo motors[4];

void gyroscope_calibrate(Euler &zero)
{
  int16_t i = 0;
  int16_t num_calibrate = 100;
  uint32_t timer = 0;
  while (i < num_calibrate)
  {
    if (timer < micros())
    {
      timer = micros() + 2000;
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);

      zero.pitch += g.gyro.x;
      zero.yaw += g.gyro.y;
      zero.roll += g.gyro.z;

      i += 1;
    }
  }

  zero.pitch /= num_calibrate;
  zero.yaw /= num_calibrate;
  zero.roll /= num_calibrate;

  // Serial.println(zero.pitch, 10);
  // Serial.println(zero.yaw, 10);
  // Serial.println(zero.roll, 10);
}

double pid_regulate(PID& K, const double err, const double dt)
{
  double proportional = err;
  K.integral += err * dt;
  double derivative = (err - K.prev_err) / dt;
  K.prev_err = err;

  return (K.p * proportional) + (K.i * K.integral) + (K.d * derivative); 
}

void setup()
{
  Serial.begin(9600);
  // Serial.println("Serial: Initialized...");
  
  BTSerial.begin(9600);

  if (!mpu.begin())
  {
    Serial.println("ERROR: Failed to find MPU6050 chip!");
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  gyroscope_calibrate(g_gyro_zero);
  delay(100);
  
  int shift = 2;
  int port = 0;
  for (int i = 0; i < 4; i += 1)
  {
    // Serial.println("Servo: attach port");
    port = shift + i;
    motors[i].attach(port);
  }

  for (int i = 0; i < 4; i += 1)
  {
    // Serial.println("Servo: setup, part 1");
    motors[i].writeMicroseconds(2300);
  }

  delay(2000);

  for (int i = 0; i < 4; i += 1)
  {
    // Serial.println("Servo: setup, part 2");
    motors[i].writeMicroseconds(800);
  }
  
  delay(6000);
}

void loop()
{
  uint32_t s_time = 0;
  uint32_t e_time = 0;

  PID K_pitch = { .p = 1.0, .i = 0.0, .d = 0.0, .prev_err = 0.0, .integral = 0.0 };
  PID K_yaw   = { .p = 1.0, .i = 0.0, .d = 0.0, .prev_err = 0.0, .integral = 0.0 };

  for (;;)
  {
    e_time = s_time;
    s_time = micros();

    double dt = (double)(s_time - e_time) / (double)1e6;
    
    // TODO(annad): User input
    float target_force = (float)BTSerial.parseInt() / 100.0f;
    // float target_force = 0.005f;
    float target_pitch = 0.0f;
    float target_yaw = 0.0f;
    
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    g_gyro_pos = {
      .pitch  = g_gyro_pos.pitch + (g.gyro.x - g_gyro_zero.pitch) * dt,
      .yaw    = g_gyro_pos.yaw + (g.gyro.y - g_gyro_zero.yaw) * dt,
      .roll   = g_gyro_pos.roll + (g.gyro.z - g_gyro_zero.roll) * dt
    };
    
    float prefer_pitch = target_pitch - g_gyro_pos.pitch;
    float prefer_yaw   = target_yaw - g_gyro_pos.yaw;

    Serial.print("prefer_yaw:");
    Serial.print(prefer_yaw, 9);
//    Serial.print(",prefer_pitch:");
//    Serial.print(prefer_pitch, 9);
    
    float force_1 = target_force *  prefer_yaw * prefer_pitch * 100000.0f;
    float force_2 = target_force * (-prefer_yaw) * prefer_pitch * 100000.0f;
    float force_3 = target_force * (-prefer_yaw) * (-prefer_pitch) * 100000.0f;
    float force_4 = target_force *  prefer_yaw * (-prefer_pitch) * 100000.0f;

//    float force_1 =   prefer_yaw * 100.0f;
//    float force_2 = (-prefer_yaw) * 100.0f;
//    float force_3 = (-prefer_yaw) * 100.0f;
//    float force_4 =   prefer_yaw * 100.0f;

    Serial.print(",force_4:");
    Serial.print(force_4);
    Serial.print(",force_2:");
    Serial.print(force_2);
    Serial.print(",force_3:");
    Serial.print(force_3);
    Serial.print(",force_4:");
    Serial.print(force_4);
    Serial.println("");

//    float result_force_1 = target_force * prefer_pitch / (2.0 * M_PI);
//    float result_force_2 = target_force * prefer_yaw / (2.0f * M_PI);
//    float result_force_3 = target_force * (-prefer_pitch) / (2.0 * M_PI);
//    float result_force_4 = target_force * (-prefer_yaw) / (2.0f * M_PI);
    int16_t throttle_1 = map(((int16_t)(constrain(force_1, -100, 100))), -100, 100, 800, 2300);
    int16_t throttle_2 = map(((int16_t)(constrain(force_2, -100, 100))), -100, 100, 800, 2300);
    int16_t throttle_3 = map(((int16_t)(constrain(force_3, -100, 100))), -100, 100, 800, 2300);
    int16_t throttle_4 = map(((int16_t)(constrain(force_4, -100, 100))), -100, 100, 800, 2300);

     motors[1-1].writeMicroseconds(throttle_1);
     motors[2-1].writeMicroseconds(throttle_2);
     motors[3-1].writeMicroseconds(throttle_3);
     motors[4-1].writeMicroseconds(throttle_4);
    
//    Serial.print("x/pitch:");
//    Serial.print(g_gyro_pos.pitch);
//    Serial.print(",y/yaw:");
//    Serial.print(g_gyro_pos.yaw);
//    Serial.print(",z/roll:");
//    Serial.println(g_gyro_pos.roll);
  }
}

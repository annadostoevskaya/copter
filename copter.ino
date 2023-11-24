/**
   File: copter.ino
   Author: github.com/annadostoevskaya
   Date: 11/10/2023 20:13:34
   Last Modified Date: 11/10/2023 20:13:43
   
   Bluetooth:
    - Serial1
    - 19 - TXD (for arduino RX)
    - 18 - RXD (for arduino TX)

   Motors:
    - pins 2, 3, 4 & 5
*/ 

#include <Servo.h>
#include <stdint.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define BTSerial Serial // 19, 18

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

double pid_regulate(const PID& K, const double err, double& prev_err, double& integral, const double dt)
{
  double proportional = err;
  integral += err * dt;
  double derivative = (err - prev_err) / dt;
  prev_err = err;

  return (K.p * proportional) + (K.i * integral) + (K.d * derivative); 
}

void setup()
{
  Serial.begin(9600);
  Serial.println("Serial: Initialized...");
  
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

void loop()
{
  uint32_t s_time = 0;
  uint32_t e_time = 0;
  int16_t throttle = 0;
  const PID K = { .p = 1.0, .i = 0.2, .d = 0.001 };
  double pid_prev_err = 0.0f;
  double pid_integral = 0.0f;

  for (;;)
  {
    e_time = s_time;
    s_time = micros();

    double dt = (double)(s_time - e_time) / (double)1e6;
    float target_force = (float)BTSerial.parseInt(); // TODO(annad): User input
    float target_pitch = 0.0f; // TODO(annad): User Input.
    
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    g_gyro_pos = {
      .pitch  = g_gyro_pos.pitch + (g.gyro.x - g_gyro_zero.pitch) * dt,
      .yaw    = g_gyro_pos.yaw + (g.gyro.y - g_gyro_zero.yaw) * dt,
      .roll   = g_gyro_pos.roll + (g.gyro.z - g_gyro_zero.roll) * dt
    };
    
    double err = target_pitch - g_gyro_pos.pitch;
    float prefer_pitch = (float)pid_regulate(K, err, pid_prev_err, pid_integral, dt);
    float result_force_1 = 0.6 * (target_force) + 0.4 * (prefer_pitch / (2.0 * M_PI));
    float result_force_3 = 0.6 * (target_force) - 0.4 * (prefer_pitch / (2.0 * M_PI));
    int16_t throttle_1 = map(((int16_t)(result_force_1 * 100.0f)), 0, 100, 800, 2300);
    int16_t throttle_3 = map(((int16_t)(result_force_3 * 100.0f)), 0, 100, 800, 2300);

    motors[1-1].writeMicroseconds(throttle_1);
    motors[3-1].writeMicroseconds(throttle_3);
    
//    Serial.print("x/pitch:");
//    Serial.print(g_gyro_pos.pitch);
//    Serial.print(",y/yaw:");
//    Serial.print(g_gyro_pos.yaw);
//    Serial.print(",z/roll:");
//    Serial.println(g_gyro_pos.roll);
  }
}

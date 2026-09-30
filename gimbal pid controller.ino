#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>

Adafruit_MPU6050 mpu;
Servo servoPitch;
Servo servoRoll;

// ---- PID gains: start here, we'll tune these live ----
float Kp = 3;
float Ki = 0.0;
float Kd = 0.0;

// ---- PID state (pitch) ----
float pitchIntegral = 0;
float pitchLastError = 0;

// ---- PID state (roll) ----
float rollIntegral = 0;
float rollLastError = 0;

unsigned long lastTime = 0;

void setup() {
  Serial.begin(115200);

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");


  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  servoPitch.attach(9);
  servoRoll.attach(10);

  lastTime = millis();
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // dt in seconds, for the D and I terms
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  if (dt <= 0) dt = 0.001;
  lastTime = now;

  // Measured tilt angles (same as before)
  float pitch = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
  float roll = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI;

  // ---- PID for pitch ----
  float pitchError = 0 - pitch;               // setpoint is 0 = level
  pitchIntegral += pitchError * dt;
  pitchIntegral = constrain(pitchIntegral, -50, 50);   // anti-windup
  float pitchDerivative = (pitchError - pitchLastError) / dt;
  pitchLastError = pitchError;

  float pitchOutput = Kp * pitchError + Ki * pitchIntegral + Kd * pitchDerivative;
  int pitchServoAngle = constrain(90 + pitchOutput, 0, 180);

  // ---- PID for roll ----
  float rollError = 0 - roll;
  rollIntegral += rollError * dt;
  rollIntegral = constrain(rollIntegral, -50, 50);
  float rollDerivative = (rollError - rollLastError) / dt;
  rollLastError = rollError;

  float rollOutput = Kp * rollError + Ki * rollIntegral + Kd * rollDerivative;
  int rollServoAngle = constrain(90 + rollOutput, 0, 180);

  servoPitch.write(pitchServoAngle);
  servoRoll.write(rollServoAngle);

  Serial.print("Pitch: "); Serial.print(pitch);
  Serial.print(" -> Servo: "); Serial.print(pitchServoAngle);
  Serial.print(" | Roll: "); Serial.print(roll);
  Serial.print(" -> Servo: "); Serial.println(rollServoAngle);
Serial.print("PitchServo:"); Serial.print(pitchServoAngle);
Serial.print(" RollServo:"); Serial.println(rollServoAngle);
  delay(20);
}

#include <Wire.h>
#include <math.h>
#include <VL53L0X.h>

VL53L0X lidar;

/*  MPU6050  */
#define SDA_PIN 21
#define SCL_PIN 22
#define MPU_ADDR 0x68

/*  MOTOR PINS  */
#define M1_PIN 26   // Front Left
#define M2_PIN 25   // Front Right
#define M3_PIN 27   // Rear Left
#define M4_PIN 14   // Rear Right

/*  RECEIVER  */
#define ROLL_PIN      32
#define PITCH_PIN     33
#define THROTTLE_PIN  34
#define YAW_PIN       35

#define STATUS_LED 2

/*  POWER  */
#define MOTOR_IDLE    1180
#define MAX_THROTTLE  1800

/*  DISTANCE  */
#define NORMAL_DISTANCE 600
#define SLOW_DISTANCE   300
#define MIN_VALID_DIST  50

/*  PWM  */
#define PWM_FREQ        50
#define PWM_RESOLUTION  16
#define PWM_PERIOD      20000

/*  PID  */
float Kp = 1.5;
float Ki = 0.02;
float Kd = 8.0;

float rollI = 0;
float pitchI = 0;

float prevRollErr = 0;
float prevPitchErr = 0;

/*  MPU DATA  */
int16_t ax, ay, az;
int16_t gx, gy, gz;

float rollAngle = 0;
float pitchAngle = 0;

unsigned long lastTime;

/*  SYSTEM  */
bool armed = false;

unsigned long armTimer = 0;
unsigned long disarmTimer = 0;

int distanceMM = 0;

void writeMotor(int ch, int pulse)
{
  pulse = constrain(pulse, 1000, 2000);

  uint32_t duty = (pulse * 65535) / PWM_PERIOD;

  ledcWrite(ch, duty);
}

void stopMotors()
{
  writeMotor(0, 1000);
  writeMotor(1, 1000);
  writeMotor(2, 1000);
  writeMotor(3, 1000);
}

void readMPU()
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU_ADDR, 14);

  if (Wire.available() == 14)
  {
    ax = Wire.read() << 8 | Wire.read();
    ay = Wire.read() << 8 | Wire.read();
    az = Wire.read() << 8 | Wire.read();

    Wire.read();
    Wire.read();

    gx = Wire.read() << 8 | Wire.read();
    gy = Wire.read() << 8 | Wire.read();
    gz = Wire.read() << 8 | Wire.read();
  }
}

void setup()
{
  Serial.begin(115200);

  pinMode(ROLL_PIN, INPUT);
  pinMode(PITCH_PIN, INPUT);
  pinMode(THROTTLE_PIN, INPUT);
  pinMode(YAW_PIN, INPUT);

  pinMode(STATUS_LED, OUTPUT);

  /*  PWM  */

  ledcSetup(0, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(2, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(3, PWM_FREQ, PWM_RESOLUTION);

  ledcAttachPin(M1_PIN, 0);
  ledcAttachPin(M2_PIN, 1);
  ledcAttachPin(M3_PIN, 2);
  ledcAttachPin(M4_PIN, 3);

  /*  I2C  */

  Wire.begin(SDA_PIN, SCL_PIN);

  delay(200);

  /*  MPU6050 INIT  */

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission();

  delay(100);

  /*  VL53L0X INIT  */

  Serial.println("Initializing VL53L0X...");

  if (!lidar.init())
  {
    Serial.println("VL53L0X FAILED!");
  }
  else
  {
    Serial.println("VL53L0X READY");

    lidar.setTimeout(500);
    lidar.setMeasurementTimingBudget(20000);

    lidar.startContinuous();
  }

  /*  ESC INIT  */

  stopMotors();

  delay(5000);

  lastTime = millis();

  Serial.println("DRONE READY");
}

void loop()
{
  /*  TIME  */

  unsigned long now = millis();

  float dt = (now - lastTime) / 1000.0;

  lastTime = now;

  if (dt <= 0 || dt > 0.05)
    dt = 0.01;

  /*  RECEIVER  */

  int rollRX     = pulseIn(ROLL_PIN, HIGH, 25000);
  int pitchRX    = pulseIn(PITCH_PIN, HIGH, 25000);
  int throttleRX = pulseIn(THROTTLE_PIN, HIGH, 25000);
  int yawRX      = pulseIn(YAW_PIN, HIGH, 25000);

  /*  LIDAR  */

  distanceMM = lidar.readRangeContinuousMillimeters();

  Serial.print("Distance: ");
  Serial.print(distanceMM);
  Serial.print(" mm");

  Serial.print(" | Armed: ");
  Serial.println(armed);

  /*  NO SIGNAL  */

  if (throttleRX == 0)
  {
    stopMotors();
    return;
  }

  /* 
                      ARM
     THROTTLE LOW + YAW RIGHT
                                           */

  if (throttleRX < 1050 && yawRX > 1900)
  {
    if (armTimer == 0)
      armTimer = millis();

    if (millis() - armTimer > 2000)
    {
      armed = true;

      rollI = 0;
      pitchI = 0;
    }
  }
  else
  {
    armTimer = 0;
  }

  /* 
                    DISARM
     THROTTLE LOW + YAW LEFT
                                           */

  if (throttleRX < 1050 && yawRX < 1100)
  {
    if (disarmTimer == 0)
      disarmTimer = millis();

    if (millis() - disarmTimer > 2000)
    {
      armed = false;
    }
  }
  else
  {
    disarmTimer = 0;
  }

  digitalWrite(STATUS_LED, armed);

  /* ===== DISARMED ===== */

  if (!armed)
  {
    stopMotors();
    return;
  }

  /* ===== BASE THROTTLE ===== */

  int base = map(throttleRX,
                 1050,
                 2000,
                 MOTOR_IDLE,
                 MAX_THROTTLE);

  /*  ANTI-CRASH LOGIC  */

  if (distanceMM > MIN_VALID_DIST)
  {
    /*  NORMAL FLIGHT  */

    if (distanceMM > NORMAL_DISTANCE)
    {
      // Full control
    }

    /*  SLOW DOWN  */

    else if (distanceMM > SLOW_DISTANCE)
    {
      base = MOTOR_IDLE + 120;
    }

    /*  SAFETY LAND  */

    else
    {
      base = MOTOR_IDLE;
    }
  }

  /*  MPU6050 STABILIZATION  */

  readMPU();

  float axg = ax / 16384.0;
  float ayg = ay / 16384.0;
  float azg = az / 16384.0;

  float rollAcc =
    atan2(axg, azg) * 180 / PI;

  float pitchAcc =
    atan2(ayg,
          sqrt(axg * axg + azg * azg))
    * 180 / PI;

  float gxds = gx / 131.0;
  float gyds = gy / 131.0;

  rollAngle =
    0.98 * (rollAngle - gyds * dt)
    + 0.02 * rollAcc;

  pitchAngle =
    0.98 * (pitchAngle + gxds * dt)
    + 0.02 * pitchAcc;

  /*  SETPOINTS  */

  float rollSet =
    map(rollRX, 1000, 2000, -20, 20);

  float pitchSet =
    map(pitchRX, 1000, 2000, -20, 20);

  /*  ERRORS  */

  float rollErr =
    rollSet - rollAngle;

  float pitchErr =
    pitchSet - pitchAngle;

  /*  INTEGRAL  */

  rollI += rollErr * dt;
  pitchI += pitchErr * dt;

  rollI = constrain(rollI, -100, 100);
  pitchI = constrain(pitchI, -100, 100);

  /*  DERIVATIVE  */

  float rollD =
    (rollErr - prevRollErr) / dt;

  float pitchD =
    (pitchErr - prevPitchErr) / dt;

  /*  PID  */

  float rollPID =
    Kp * rollErr +
    Ki * rollI +
    Kd * rollD;

  float pitchPID =
    Kp * pitchErr +
    Ki * pitchI +
    Kd * pitchD;

  prevRollErr = rollErr;
  prevPitchErr = pitchErr;

  /*   MOTOR MIXING   */

  int m1 =
    base + pitchPID + rollPID;

  int m2 =
    base + pitchPID - rollPID;

  int m3 =
    base - pitchPID + rollPID;

  int m4 =
    base - pitchPID - rollPID;

  /*  SAFETY LIMITS  */

  m1 = constrain(m1, 1000, 2000);
  m2 = constrain(m2, 1000, 2000);
  m3 = constrain(m3, 1000, 2000);
  m4 = constrain(m4, 1000, 2000);

  /*  WRITE MOTORS  */

  writeMotor(0, m1);
  writeMotor(1, m2);
  writeMotor(2, m3);
  writeMotor(3, m4);

  delay(5);
}

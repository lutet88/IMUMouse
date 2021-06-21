#include <Adafruit_LSM6DS33.h>
#include <Adafruit_Sensor_Calibration.h>
#include <Adafruit_AHRS.h>

#define ZERO_BUTTON 7

const int sensorFreq = 208;
const int sensorTimingMilliseconds = (int) ((1.0 / (float) sensorFreq) * 1000);
double orientation[] = {0.0, 0.0, 0.0};
double baseline[] = {0.0, 0.0, 0.0};
double baseline_gyro[] = {0.0, 0.0, 0.0};
double velocity[] = {0.0, 0.0, 0.0};
double pos[] = {0.0, 0.0, 0.0};
unsigned long prev_time = millis();
const double rad_to_deg = 57.2957795131;

Adafruit_LSM6DS33 lsm6ds33;
Adafruit_Sensor_Calibration_SDFat cal;

void zero() {
  orientation[0] = 0.0;
  orientation[1] = 0.0;
  orientation[2] = 0.0;
  velocity[0] = 0.0;
  velocity[1] = 0.0;
  velocity[2] = 0.0;
  pos[0] = 0.0;
  pos[1] = 0.0;
  pos[2] = 0.0;
  setBaseline();
}

void setBaseline(){
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  lsm6ds33.getEvent(&accel, &gyro, &temp);

  baseline[0] = accel.acceleration.x;
  baseline[1] = accel.acceleration.y;
  baseline[2] = accel.acceleration.z;

  baseline_gyro[0] = gyro.gyro.x;
  baseline_gyro[1] = gyro.gyro.y;
  baseline_gyro[2] = gyro.gyro.z;
}

void setup() {
  pinMode(ZERO_BUTTON, INPUT_PULLUP);
  Serial.begin(115200);
  while (!Serial) {delay(10);}

  cal.begin();
  cal.loadCalibration();
  lsm6ds33.begin_I2C();

  // most accurate accel/gyro ranges
  lsm6ds33.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  lsm6ds33.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);

  // since USB HID works at 125Hz, we will use 208Hz
  lsm6ds33.setAccelDataRate(LSM6DS_RATE_208_HZ);
  lsm6ds33.setGyroDataRate(LSM6DS_RATE_208_HZ);

  // i have no idea what these actually do, but they seem important?
  lsm6ds33.configInt1(false, false, true); // accelerometer DRDY on INT1
  lsm6ds33.configInt2(false, true, false); // gyro DRDY on INT2

  setBaseline();
}

void loop() {
  if (digitalRead(ZERO_BUTTON) == LOW) { 
    zero();
  } else {
    // get delta time
    unsigned long current_time = millis();
    unsigned long elapsed_time = current_time - prev_time;
    if (elapsed_time >= sensorTimingMilliseconds) {
      // it's been at least 1/208s
      prev_time = current_time;
      
      sensors_event_t accel;
      sensors_event_t gyro;
      sensors_event_t temp;
      lsm6ds33.getEvent(&accel, &gyro, &temp);

      cal.calibrate(accel);
      cal.calibrate(gyro);

      // measured in rad/s, convert to degrees
      orientation[0] += ((gyro.gyro.x - baseline_gyro[0]) * rad_to_deg) / sensorFreq;
      orientation[1] += ((gyro.gyro.y - baseline_gyro[1]) * rad_to_deg) / sensorFreq;
      orientation[2] += ((gyro.gyro.z - baseline_gyro[2]) * rad_to_deg) / sensorFreq;

      // measured in m/s^2, get velocity vector
      velocity[0] += ((accel.acceleration.x - baseline[0]) / sensorFreq);
      velocity[1] += ((accel.acceleration.y - baseline[1]) / sensorFreq);
      velocity[2] += ((accel.acceleration.z - baseline[2]) / sensorFreq);

      
      Serial.print("\t\tOrientation Z: ");
      Serial.print(orientation[2]);
      Serial.println(" degrees ");

      
      Serial.print("\t\tVelocity X: ");
      Serial.print(velocity[0]);
      Serial.print(" \tY: ");
      Serial.print(velocity[1]);
      Serial.println(" m/s ");
    }
  }
}

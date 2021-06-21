#include <SimpleKalmanFilter.h>
#include <Adafruit_LSM6DS33.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor_Calibration.h>
#include <Adafruit_AHRS.h>

#define ZERO_BUTTON 7
#define FREQUENCY 208

const int sensorTimingMilliseconds = (int) ((1.0 / (float) FREQUENCY) * 1000);
unsigned long prev_time = millis();
const double r2d = 57.2957795131;
double accel_zero[] = {0.0, 0.0, 0.0};

Adafruit_LSM6DS33 lsm6ds33;
Adafruit_LIS3MDL lis3mdl;
Adafruit_Sensor_Calibration_SDFat cal;
Adafruit_Madgwick filter;
Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;

int counter;

SimpleKalmanFilter kfx = SimpleKalmanFilter(2, 2, 0.01);
SimpleKalmanFilter kfy = SimpleKalmanFilter(2, 2, 0.01);
SimpleKalmanFilter kfz = SimpleKalmanFilter(2, 2, 0.01);

void zero() {
  sensors_event_t accel;
  accelerometer->getEvent(&accel);
  
  accel_zero[0] = kfx.updateEstimate(accel.acceleration.x);
  accel_zero[1] = kfy.updateEstimate(accel.acceleration.y);
  accel_zero[2] = kfz.updateEstimate(accel.acceleration.z);
}

void setup() {
  pinMode(ZERO_BUTTON, INPUT_PULLUP);
  Serial.begin(115200);
  while (!Serial) {delay(10);}

  cal.begin();
  cal.loadCalibration();
  lsm6ds33.begin_I2C();
  lis3mdl.begin_I2C();

  // most accurate accel/gyro/mag ranges
  lsm6ds33.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  lsm6ds33.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);

  // since USB HID works at 125Hz, we will use 208Hz
  lsm6ds33.setAccelDataRate(LSM6DS_RATE_208_HZ);
  lsm6ds33.setGyroDataRate(LSM6DS_RATE_208_HZ);

  // magnetometer
  lis3mdl.setDataRate(LIS3MDL_DATARATE_1000_HZ);
  lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);

  // i have no idea what these actually do, but they seem important?
  lsm6ds33.configInt1(false, false, true); // accelerometer DRDY on INT1
  lsm6ds33.configInt2(false, true, false); // gyro DRDY on INT2

  accelerometer = lsm6ds33.getAccelerometerSensor();
  gyroscope = lsm6ds33.getGyroSensor();
  magnetometer = &lis3mdl;

  filter.begin(FREQUENCY);
  Wire.setClock(400000);

  zero();
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
      prev_time = millis();
      
      sensors_event_t accel, gyro, mag;
      accelerometer->getEvent(&accel);
      gyroscope->getEvent(&gyro);
      magnetometer->getEvent(&mag);

      cal.calibrate(accel);
      cal.calibrate(gyro);
      cal.calibrate(mag);

      filter.update(gyro.gyro.x * r2d, gyro.gyro.y * r2d, gyro.gyro.z * r2d, 
                accel.acceleration.x, accel.acceleration.y, accel.acceleration.z, 
                mag.magnetic.x, mag.magnetic.y, mag.magnetic.z);

      double yaw = filter.getYaw();
      double accelX = kfx.updateEstimate(accel.acceleration.x);
      double accelY = kfy.updateEstimate(accel.acceleration.y);
      double accelZ = kfz.updateEstimate(accel.acceleration.z);

      counter ++;
      if (counter % 25 == 0){
        Serial.print("\t\tYaw: ");
        Serial.print(yaw);
        Serial.println(" degrees ");
        
        Serial.print("\t\tAccel X: ");
        Serial.print(1000 * (accelX - accel_zero[0]));
        Serial.print(" \tY: ");
        Serial.print(1000 * (accelY - accel_zero[1]));
        Serial.print(" \tZ: ");
        Serial.print(1000 * (accelZ - accel_zero[2]));
        Serial.println(" m/s ");
      }
      if (counter % 1000 == 0){
        zero();
        Serial.println("Zeroing!");
      }
    }
  }
}

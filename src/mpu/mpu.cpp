#include <./mpu/mpu.hpp>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <./include/globals.hpp>


// #define X 0 // pitch
// #define Y 1 // roll
// #define Z 2 // yaw
#define FREQ 250 // Sampling frequency

float gyro_timer;

unsigned long int loop_timer;
unsigned long int now; // Exists just to reduce the calls to micros()
int period;

float gyro_angle[3]  = {0,0,0};
float gyro_rate[3]  = {0,0,0};
float gyro_offset[3] = {0,0,0};

float acc_angle[3] = {0,0,0};
float acc_total_vector;

float pitch_angle;
float roll_angle;
float yaw_angle;

float filter_ratio = 0.9996; //what ratio of gyro angle to accelerometer angle;



Adafruit_MPU6050 mpu;

sensors_event_t a, g, temp; //the variables read from the sensor. a = acceleration, g = gyro and temp = temperature

void configureMPU () {
    now = micros();

  // Calculate period in µs
  period = (1000000/FREQ);

  // Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  // Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  // Serial.print("Accelerometer range set to: ");
  // switch (mpu.getAccelerometerRange()) {
  // case MPU6050_RANGE_2_G:
  //   Serial.println("+-2G");
  //   break;
  // case MPU6050_RANGE_4_G:
  //   Serial.println("+-4G");
  //   break;
  // case MPU6050_RANGE_8_G:
  //   Serial.println("+-8G");
  //   break;
  // case MPU6050_RANGE_16_G:
  //   Serial.println("+-16G");
  //   break;
  // }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  // Serial.print("Gyro range set to: ");
  // switch (mpu.getGyroRange()) {
  // case MPU6050_RANGE_250_DEG:
  //   Serial.println("+- 250 deg/s");
  //   break;
  // case MPU6050_RANGE_500_DEG:
  //   Serial.println("+- 500 deg/s");
  //   break;
  // case MPU6050_RANGE_1000_DEG:
  //   Serial.println("+- 1000 deg/s");
  //   break;
  // case MPU6050_RANGE_2000_DEG:
  //   Serial.println("+- 2000 deg/s");
  //   break;
  // }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  // Serial.print("Filter bandwidth set to: ");
  // switch (mpu.getFilterBandwidth()) {
  // case MPU6050_BAND_260_HZ:
  //   Serial.println("260 Hz");
  //   break;
  // case MPU6050_BAND_184_HZ:
  //   Serial.println("184 Hz");
  //   break;
  // case MPU6050_BAND_94_HZ:
  //   Serial.println("94 Hz");
  //   break;
  // case MPU6050_BAND_44_HZ:
  //   Serial.println("44 Hz");
  //   break;
  // case MPU6050_BAND_21_HZ:
  //   Serial.println("21 Hz");
  //   break;
  // case MPU6050_BAND_10_HZ:
  //   Serial.println("10 Hz");
  //   break;
  // case MPU6050_BAND_5_HZ:
  //   Serial.println("5 Hz");
  //   break;
  // }

  calibrateMpu6050();

  // Serial.println("");
  delay(100);
}

void readSensor(){
  mpu.getEvent(&a, &g, &temp); //get the values from the MPU6050

  gyro_rate[X] = g.gyro.x - gyro_offset[X];
  gyro_rate[Y] = g.gyro.x - gyro_offset[Y];
  gyro_rate[Z] = g.gyro.x - gyro_offset[Z];
}

void calibrateMpu6050()
{
  // Serial.println("Calibrating gyro...");
    int max_samples = 2000;
    
    for (int i = 0; i < max_samples; i++) {
        readSensor();

        gyro_offset[X] += g.gyro.x;
        gyro_offset[Y] += g.gyro.y;
        gyro_offset[Z] += g.gyro.z;

        // Just wait a bit before next loop
        delay(3);
    }

    // Calculate average offsets
    gyro_offset[X] /= max_samples;
    gyro_offset[Y] /= max_samples;
    gyro_offset[Z] /= max_samples;

    // Serial.println("Gyro calibrated. Safe flight!");
      
}

void calculateGyroAngles()
{
  // Angle calculation
  gyro_angle[X] += (g.gyro.x - gyro_offset[X]) * (180 / (FREQ * PI)); //g.gyro return in rad/s, you multipy by FREQ to have it in rad for the 
                                                                      // required period, and convert it to degrees
  gyro_angle[Y] += (g.gyro.y - gyro_offset[Y]) * (180 / (FREQ * PI));

  // Transfer roll to pitch if IMU has yawed
  gyro_angle[Y] -= (gyro_angle[X] * sin((g.gyro.z - gyro_offset[Z])/(FREQ))) * (PI / 180); //convert angle to rad first, same comment as before for the * by FREQ
  gyro_angle[X] += (gyro_angle[Y] * sin((g.gyro.z - gyro_offset[Z])/(FREQ))) * (PI / 180);

  gyro_angle[Z] += (g.gyro.z - gyro_offset[Z]) * (180 / (FREQ * PI));

}

void measureAccelerometerAngles()
{
  // Calculate total 3D acceleration vector : √(X² + Y² + Z²)
  acc_total_vector = sqrt(pow(a.acceleration.x, 2) + pow(a.acceleration.y, 2) + pow(a.acceleration.z, 2));

  // To prevent asin to produce a NaN, make sure the input value is within [-1;+1]
  if (abs(a.acceleration.x) < acc_total_vector) {
    acc_angle[X] = asin((float)a.acceleration.y / acc_total_vector) * (180 / PI); // asin gives angle in radian. Convert to degree multiplying by 180/pi
  }

  if (abs(a.acceleration.y) < acc_total_vector) {
    acc_angle[Y] = asin((float)a.acceleration.x / acc_total_vector) * (180 / PI);
  }
}


void filterAnglesKalman()
{
  pitch_angle = (gyro_angle[X] * filter_ratio) + (acc_angle[X] * (1 - filter_ratio));
  roll_angle = (gyro_angle[Y] * filter_ratio) + (acc_angle[Y] * (1 - filter_ratio));
  yaw_angle = gyro_angle[Z];
}

void measureAcceleration() {

}

void measureSpeed() {
    
}

void displayMPUValues () {
/* Print out the values */
//  Serial.print("Acceleration X: ");
//  Serial.print(a.acceleration.x);
//  Serial.print(", Y: ");
//  Serial.print(a.acceleration.y);
//  Serial.print(", Z: ");
//  Serial.print(a.acceleration.z);
//  Serial.println(" m/s^2");

//  Serial.print("Rotation X: ");
//  Serial.print((g.gyro.x - gyro_offset[X]) * (180/PI));
//  Serial.print(", Y: ");
//  Serial.print((g.gyro.y - gyro_offset[Y]) * (180/PI));
//  Serial.print(", Z: ");
//  Serial.print((g.gyro.z - gyro_offset[Z]) * (180/PI));
//  Serial.println(" deg/s");
//
//    Serial.print("Acc Angle X: ");
//  Serial.print(acc_angle[X]);
//  Serial.print(", Y: ");
//  Serial.print(acc_angle[Y]);
//  Serial.print(", Z: ");
//  Serial.print(acc_angle[Z]);
//  Serial.println(" deg");

//      Serial.print("Gyro Angle X: ");
//  Serial.print(gyro_angle[X]);
//  Serial.print(", Y: ");
//  Serial.print(gyro_angle[Y]);
//  Serial.print(", Z: ");
//  Serial.print(gyro_angle[Z]);
//  Serial.println(" deg");
//
      Serial.print("Pitch: ");
  Serial.print(pitch_angle);
  Serial.print(", Roll: ");
  Serial.print(roll_angle);
  Serial.print(", Yaw: ");
  Serial.println(yaw_angle);

}
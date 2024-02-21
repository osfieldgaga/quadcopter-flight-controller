#ifndef MPU
#define MPU

extern float gyro_angle[3];
extern float gyro_rate[3];
extern float gyro_offset[3];

extern float pitch_angle;
extern float roll_angle;
extern float yaw_angle;

void configureMPU();
void readSensor();
void calibrateMpu6050();
void calculateGyroAngles();
void measureAccelerometerAngles();
void filterAnglesKalman();



#endif
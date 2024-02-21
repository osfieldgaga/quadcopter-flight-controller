#include <./pid/pid.hpp>
#include <./mpu/mpu.hpp>
#include <./include/config.hpp>
#include <./include/utils.hpp>
#include <./receiver/receiver.hpp>
#include <./include/globals.hpp>

/* -------------------------------------------------------------------------- */
/*                                 Rate mode                                 */
/* -------------------------------------------------------------------------- */
float inputRoll, inputPitch, inputYaw;

float errorRoll, errorPitch, errorYaw;

float prevErrorRoll, prevErrorPitch, prevErrorYaw;

float prevItermRoll, prevItermPitch, prevItermYaw;

float PIDOutput[3] = {0, 0, 0};

float P_Roll = 0.6;
float P_Pitch = P_Roll;
float P_Yaw = 2;

float I_Roll = 3.5;
float I_Pitch = I_Roll;
float I_Yaw = 12;

float D_Roll = 0.03;
float D_Pitch = D_Roll;
float D_Yaw = 0;

/* -------------------------------------------------------------------------- */
/*                                 Angle mode                                 */
/* -------------------------------------------------------------------------- */
float inputRollAngle, inputPitchAngle, inputYawAngle;

float errorRollAngle, errorPitchAngle;

float prevErrorRollAngle, prevErrorPitchAngle;

float prevItermRollAngle, prevItermPitchAngle;

float PIDOutputAngle[3] = {0, 0, 0};

float P_Roll_Angle = 2;
float P_Pitch_Angle = P_Roll_Angle;

float I_Roll_Angle = 0;
float I_Pitch_Angle = I_Roll_Angle;

float D_Roll_Angle = 0;
float D_Pitch_Angle = D_Roll_Angle;

float motorInput1, motorInput2, motorInput3, motorInput4;

void PIDEquation(float error, float P, float I, float D, float prevError, float prevIterm)

{
  float Pterm = P * error;
  float Iterm = prevIterm + I * ((error * prevError) * 0.004) / 2;
  Iterm = minMax(Iterm, -400, 400);

  float Dterm = D * (error - prevError) / 0.004;

  float PID_out = Pterm + Iterm + Dterm;

  PIDOutput[0] = minMax(PID_out, -400, 400);
  PIDOutput[1] = error;
  PIDOutput[2] = Iterm;

  // return minMax(PID_out, -400, 400);
}

void PID_Roll()
{
  PIDEquation(errorRoll, P_Roll, I_Roll, D_Roll, prevErrorRoll, prevItermRoll);
  inputRoll = PIDOutput[0];
  prevErrorRoll = PIDOutput[1];
  prevItermRoll = PIDOutput[2];
}

void PID_Pitch()
{
  PIDEquation(errorPitch, P_Pitch, I_Pitch, D_Pitch, prevErrorPitch, prevItermPitch);
  inputPitch = PIDOutput[0];
  prevErrorPitch = PIDOutput[1];
  prevItermPitch = PIDOutput[2];
}

void PID_Yaw()
{
  PIDEquation(errorYaw, P_Yaw, I_Yaw, D_Yaw, prevErrorYaw, prevItermYaw);
  inputYaw = PIDOutput[0];
  prevErrorYaw = PIDOutput[1];
  prevItermYaw = PIDOutput[2];
}

void PID_Controller_Rate()
{
  PID_Roll();
  PID_Pitch();
  PID_Yaw();
}

void PID_Controller_Angle()
{

// roll
  PIDEquation(errorRollAngle, P_Roll_Angle, I_Roll_Angle, D_Roll_Angle, prevErrorRollAngle, prevItermRollAngle);
  desiredRoll = PIDOutputAngle[0];
  prevErrorRollAngle = PIDOutput[1];
  prevItermRollAngle = PIDOutput[2];

// pitch
  PIDEquation(errorPitchAngle, P_Pitch_Angle, I_Pitch_Angle, D_Pitch_Angle, prevErrorPitchAngle, prevItermPitchAngle);
  desiredPitch = PIDOutputAngle[0];
  prevErrorPitchAngle = PIDOutputAngle[1];
  prevItermPitchAngle = PIDOutputAngle[2];

  PID_Yaw();
}

void resetPID()
{
  prevErrorRoll = 0;
  prevErrorPitch = 0;
  prevErrorYaw = 0;
  prevItermRoll = 0;
  prevItermPitch = 0;
  prevItermYaw = 0;
}

void calculateErrors()
{
  errorRoll = desiredRoll - gyro_rate[Y];   // Y axis is roll
  errorPitch = desiredPitch - gyro_rate[X]; // X axis is pitch
  errorYaw = desiredYaw - gyro_rate[Z];     // Z axis is yaw

  errorRollAngle = desiredRollAngle - roll_angle;    // Y axis is roll
  errorPitchAngle = desiredPitchAngle - pitch_angle; // X axis is pitch
                                                     // Z axis is yaw
}

float calculateSetPoints(float current_measurement, float target)

{
  float set_point = 0;
  float K_t = 3;
  float K_a = 15;

  int margin = 8; // how many us around the center of the joystick we allow as dead zone

  int pwm_mid = ((HIGH_PWM - LOW_PWM) / 2); // probably 1500us. The mid point is when the joystick is at the center for that axis

  if (target > pwm_mid + margin)
  {
    set_point = target - (pwm_mid + margin);
  }
  else if (target < pwm_mid - margin)
  {
    set_point = target - (pwm_mid - margin);
  }

  set_point -= (current_measurement * K_a) / K_t;

  return set_point;
}
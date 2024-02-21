#ifndef PID
#define PID

extern float PIDOutput[3];
extern float PIDOutputAngle[3];
extern float errorRoll, errorPitch, errorYaw;
extern float inputRoll, inputPitch, inputYaw;
extern float errorRollAngle, errorPitchAngle, errorYawAngle;
extern float inputRollAngle, inputPitchAngle, inputYawAngle;

void calculateErrors();
void PID_Controller_Rate();
void PID_Controller_Angle();
void resetPID();

#endif
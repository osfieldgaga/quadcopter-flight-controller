#ifndef PID
#define PID

extern float PIDOutput[3];
extern float errorRoll, errorPitch, errorYaw;
extern float inputRoll, inputPitch, inputYaw;

void calculateErrors();
void PID_Controller();

#endif
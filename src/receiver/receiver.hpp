#ifndef RECEIVER
#define RECEIVER

extern float inputThrottle, desiredPitch, desiredRoll, desiredYaw;
extern float desiredPitchAngle, desiredRollAngle;

void readReceiverRateMode();
void readReceiverAngleMode();



#endif
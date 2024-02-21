#include <./receiver/receiver.hpp>
#include <./include/utils.hpp>

enum Channels {
    L_X, //yaw
    L_Y, //throttle
    R_X, //roll
    R_Y//pitch
};

float receiverValues[4] = {0, 0, 0, 0};

// float receivedThrottle, receivedPitch, receivedRoll, receivedYaw;
float inputThrottle, desiredPitch, desiredRoll, desiredYaw;

float desiredPitchAngle, desiredRollAngle;


void readReceiverRateMode() {


    desiredRoll = 0.15 * (receiverValues[R_X] - 1500);
    desiredPitch = 0.15 * (receiverValues[R_Y] - 1500);

    inputThrottle = receiverValues[L_Y];
    if(inputThrottle > 1800) inputThrottle = 1800;

    desiredYaw = 0.15 * (receiverValues[L_X] - 1500);
}

void readReceiverAngleMode() {


    desiredRollAngle = 0.1 * (receiverValues[R_X] - 1500);
    desiredPitchAngle = 0.1 * (receiverValues[R_Y] - 1500);

    inputThrottle = receiverValues[L_Y];
    if(inputThrottle > 1800) inputThrottle = 1800;

    desiredYaw = 0.15 * (receiverValues[L_X] - 1500);
}
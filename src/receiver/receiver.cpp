#include <./receiver/receiver.hpp>
#include <./include/utils.hpp>
#include <RH_ASK.h>
#ifdef RH_HAVE_HARDWARE_SPI
#include <SPI.h> // Not actually used but needed to compile
#endif

#include <string.h>

enum Channels
{
    L_X, // yaw
    L_Y, // throttle
    R_X, // roll
    R_Y  // pitch
};

float receiverValues[4] = {0, 0, 0, 0};

// float receivedThrottle, receivedPitch, receivedRoll, receivedYaw;
float inputThrottle, desiredPitch, desiredRoll, desiredYaw;

float desiredPitchAngle, desiredRollAngle;

RH_ASK driver(2000, D9, D4, D10);

void readReceiverRateMode()
{

    reacController();

    desiredRoll = 0.15 * (receiverValues[R_X] - 1500);
    desiredPitch = 0.15 * (receiverValues[R_Y] - 1500);

    inputThrottle = receiverValues[L_Y];
    if (inputThrottle > 1800)
        inputThrottle = 1800;

    desiredYaw = 0.15 * (receiverValues[L_X] - 1500);
}

void readReceiverAngleMode()
{

    reacController();

    desiredRollAngle = 0.1 * (receiverValues[R_X] - 1500);
    desiredPitchAngle = 0.1 * (receiverValues[R_Y] - 1500);

    inputThrottle = receiverValues[L_Y];
    if (inputThrottle > 1800)
        inputThrottle = 1800;

    desiredYaw = 0.15 * (receiverValues[L_X] - 1500);
}

void reacController()
{
    uint8_t buf[RH_ASK_MAX_MESSAGE_LEN];
    uint8_t buflen = sizeof(buf);

    if (driver.recv(buf, &buflen)) // Non-blocking
    {
        int i;

        // Message with a good checksum received, dump it.
        driver.printBuffer("Got:", buf, buflen);
    }

    char *rcv;

    for (int i = 0; i < buflen; i++)
    {
        rcv += (char)buf[i];
    }

    // Tokenize the string based on commas
    char *token = strtok(rcv, ",");

    // Variables to store the parsed values

    // Parse and store the values
    if (token != nullptr)
    {
        receiverValues[L_Y] = atoi(token);
        // throttle = token;
        token = strtok(nullptr, ",");
    }
    if (token != nullptr)
    {
        receiverValues[R_Y] = atoi(token);
        // pitch = token;
        token = strtok(nullptr, ",");
    }
    if (token != nullptr)
    {
        receiverValues[L_X] = atoi(token);
        // yaw = token;
        token = strtok(nullptr, ",");
    }
    if (token != nullptr)
    {
        receiverValues[R_X] = atoi(token);
        // roll = token;
    }
}
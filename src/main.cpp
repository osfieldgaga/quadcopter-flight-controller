#include <Arduino.h>


#include <./mpu/mpu.hpp>
#include <./pid/pid.hpp>
#include <./include/config.hpp>
#include <./include/globals.hpp>
#include <./include/utils.hpp>
#include <./receiver/receiver.hpp>

enum States
{
  STOPPED = 0,
  ARMED = 1,
  STARTED = 2
};

enum FlightMode
{
  RATE = 0,
  ANGLE = 1
};

int currentState;
int flightMode;

// // put function declarations here:
void readControllerOverSerial();
void recvWithStartEndMarkers();
void parseData();
void setMotorSpeed(int motorID, float throttle);
void setAllMotorsSpeed(float throttle);
void calculateMotorsInput();

int potpin = 0; // analog pin used to connect the potentiometer
int val;        // variable to read the value from the analog pin

float motorInput[4];

int motorPins[NUMBER_OF_MOTORS] = {D5, D6, D7, D8};
int statusLED = D4;
int statusTimer = 500; // blink every 500ms
int statusNow;

bool countingDown = false;

int stoppedCommandsTimer = 3000;
int stoppedNow = 0;

int armedCommandsTimer = 3000;
int armedNow = 0;

const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars]; // temporary array for use when parsing

boolean newData = false;

// variables to hold received data
float throttle, pitch, yaw, roll;

void setup()
{
  // put your setup code here, to run once:
  currentState = STOPPED;
  flightMode = RATE;
  statusNow = millis();
  armedNow = millis();
  Serial.begin(9600);

  pinMode(statusLED, OUTPUT);
  digitalWrite(statusLED, HIGH);

  delay(500); // give some time to the sensor to start
  configureMPU();

  resetPID();
}

void loop()
{
  // lcd.setCursor(0, 0);

  recvWithStartEndMarkers();

  if (newData == true)
  {
    strcpy(tempChars, receivedChars);
    // this temporary copy is necessary to protect the original data
    //   because strtok() used in parseData() replaces the commas with \0
    parseData();

    newData = false;
  }

  switch (currentState)
  {

    /* -------------------------------------------------------------------------- */
    /*                              Drone is  stopped                             */
    /* -------------------------------------------------------------------------- */
  case STOPPED:
    digitalWrite(statusLED, HIGH);

    /* --------------------- CHeck if arming command is sent -------------------- */
    if (throttle <= 20)
    {

      if (!countingDown)
      {
        stoppedNow = millis();
        countingDown = true;
      }
      else
      {
        if (millis() - stoppedNow > stoppedCommandsTimer)
        {
          stoppedNow = millis();
          countingDown = false;
          currentState = ARMED;
        }
      }
    }
    else
    {
      countingDown = false;
    }

    break;

    /* -------------------------------------------------------------------------- */
    /*                            Drone is armed and ON                           */
    /* -------------------------------------------------------------------------- */
  case ARMED:
    // show data on LED or motors

    if (millis() - statusNow > statusTimer)
    {

      if (digitalRead(statusLED) == HIGH)
      {
        digitalWrite(statusLED, LOW);
      }
      else
      {
        digitalWrite(statusLED, HIGH);
      }

      statusNow = millis();
    }

    /* -------------------- Check if starting command is sent ------------------- */

    if (throttle >= 1000)
    {
      if (!countingDown)
      {
        armedNow = millis();
        countingDown = true;
      }
      else
      {
        if (millis() - armedNow > armedCommandsTimer)
        {
          armedNow = millis();
          countingDown = false;
          currentState = STARTED;
        }
      }
    }
    else
    {
      countingDown = false;
    }

    break;

    /* -------------------------------------------------------------------------- */
    /*                               Drone is flying                              */
    /* -------------------------------------------------------------------------- */
  case STARTED:

    digitalWrite(statusLED, LOW);

    readSensor();

    switch (flightMode)
    {
    case RATE:
      readReceiverRateMode();
      break;
    case ANGLE:
    // commands set desired angles fo each axis
      readReceiverAngleMode();
      break;
    default:
      break;
    }

    desiredRoll = 0.15 * (map(roll, 0, 1023, 1000, 2000) - 1500);
    desiredPitch = 0.15 * (map(pitch, 0, 1023, 1000, 2000) - 1500);

    inputThrottle = map(throttle, 0, 1023, 1000, 2000);
    if (inputThrottle > 1800)
      inputThrottle = 1800;

    desiredYaw = 0.15 * (map(yaw, 0, 1023, 1000, 2000) - 1500);

    calculateErrors();

    if (flightMode == ANGLE)
      PID_Controller_Angle();

    PID_Controller_Rate();

    calculateMotorsInput();

    // angle mode

    // calculateGyroAngles();
    // measureAccelerometerAngles();
    // filterAnglesKalman();

    Serial.println("Commands:");
    Serial.print("T: ");
    Serial.print(inputThrottle);
    Serial.print(", R: ");
    Serial.print(desiredRoll);
    Serial.print(", P: ");
    Serial.print(desiredPitch);
    Serial.print(", Y: ");
    Serial.println(desiredYaw);
    Serial.println();

    Serial.println("Errors:");
    Serial.print("R: ");
    Serial.print(errorRoll);
    Serial.print(", P: ");
    Serial.print(errorPitch);
    Serial.print(", Y: ");
    Serial.print(errorYaw);

    Serial.println("Motors:");
    Serial.print("M1: ");
    Serial.print(motorInput[0]);
    Serial.print(", M2: ");
    Serial.print(motorInput[1]);
    Serial.print(", M3: ");
    Serial.print(motorInput[2]);
    Serial.print(", M4: ");
    Serial.println(motorInput[3]);
    Serial.println();

    // Do nothing until sampling period is reached
    while ((now = micros()) - loop_timer < period)
      ;
    loop_timer = now;

    break;
  }
}

//============

void recvWithStartEndMarkers()
{
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  while (Serial.available() > 0 && newData == false)
  {
    rc = Serial.read();

    if (recvInProgress == true)
    {
      if (rc != endMarker)
      {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars)
        {
          ndx = numChars - 1;
        }
      }
      else
      {
        receivedChars[ndx] = '\0'; // terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    }

    else if (rc == startMarker)
    {
      recvInProgress = true;
    }
  }
}

//============

void parseData()
{ // split the data into its parts

  // Tokenize the string based on commas
  char *token = strtok(tempChars, ",");

  // Variables to store the parsed values

  // Parse and store the values
  if (token != nullptr)
  {
    throttle = atoi(token);
    token = strtok(nullptr, ",");
  }
  if (token != nullptr)
  {
    pitch = atoi(token);
    token = strtok(nullptr, ",");
  }
  if (token != nullptr)
  {
    yaw = atoi(token);
    token = strtok(nullptr, ",");
  }
  if (token != nullptr)
  {
    roll = atoi(token);
  }
}

void setMotorSpeed(int motorPin, float input)
{
  analogWriteFreq(250);
  analogWriteResolution(12);

  analogWrite(motorPin, input);
}

void calculateMotorsInput()
{
  motorInput[0] = 1.024 * (inputThrottle - inputRoll - inputPitch - inputYaw);
  motorInput[1] = 1.024 * (inputThrottle - inputRoll + inputPitch + inputYaw);
  motorInput[2] = 1.024 * (inputThrottle + inputRoll + inputPitch - inputYaw);
  motorInput[3] = 1.024 * (inputThrottle + inputRoll - inputPitch + inputYaw);

  // avoid overloading the motors by limiting it to 1999
  // don't allow motors to stop mid flight by setting min throttle > 1000
  int minThrottle = 1180;

  motorInput[0] = minMax(motorInput[0], minThrottle, 1999);
  motorInput[1] = minMax(motorInput[1], minThrottle, 1999);
  motorInput[2] = minMax(motorInput[2], minThrottle, 1999);
  motorInput[3] = minMax(motorInput[3], minThrottle, 1999);

  int i;
  for (i = 0; i < NUMBER_OF_MOTORS; i++)
  {
    setMotorSpeed(motorPins[i], motorInput[i]);
  }
}

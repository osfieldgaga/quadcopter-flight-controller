#include <Arduino.h>
#include <Servo.h>
#include <LiquidCrystal.h>

#define NUMBER_OF_MOTORS 4
#define LOW_PWM 1000
#define HIGH_PWM 20000

enum States
{
  STOPPED = 0,
  ARMED = 1,
  STARTED = 2
};

int currentState;

// // put function declarations here:
void readControllerOverSerial();
void recvWithStartEndMarkers();
void parseData();
void setMotorSpeed(int motorID, float throttle);
void setAllMotorsSpeed(float throttle);

int potpin = 0; // analog pin used to connect the potentiometer
int val;        // variable to read the value from the analog pin

Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;

Servo motor[NUMBER_OF_MOTORS];
int motorPins[NUMBER_OF_MOTORS] = {D5, D6, D7, D8};
int statusLED = D4;
int statusTimer = 500; // blink every 500ms
int statusNow;

bool countingDown = false;

int stoppedCommandsTimer = 3000;
int stoppedNow = 0;

int armedCommandsTimer = 3000;
int armedNow = 0;

// int rs = 7;
// int en = 8;
// int d4 = 9;
// int d5 = 10;
// int d6 = 11;
// int d7 = 12;
// LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

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
  statusNow = millis();
  armedNow = millis();
  Serial.begin(9600);

  pinMode(statusLED, OUTPUT);
  digitalWrite(statusLED, HIGH);
  
  motor1.attach(D5, 1000, 20000);
  motor2.attach(D6, 1000, 20000);
  motor3.attach(D7, 1000, 20000);
  motor4.attach(D8, 1000, 20000);

  // int i = 0;

  // for (i = 0; i < NUMBER_OF_MOTORS; i++)
  // {
  //   motor[i].attach(motorPins[i], LOW_PWM, HIGH_PWM);
  // }

  // pinMode(13, OUTPUT);
  // lcd.begin(16, 2);
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
    motor1.write(0);
    motor2.write(0);
    motor3.write(0);
    motor4.write(0);
    // digitalWrite(statusLED, HIGH);
    // digitalWrite(LED_BUILTIN, HIGH);
    // setAllMotorsSpeed(0.1);

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
    // show data on LED or motors
    motor1.write(map(throttle, 0, 1023, 0, 180));
    motor2.write(map(throttle, 0, 1023, 0, 180));
    motor3.write(map(throttle, 0, 1023, 0, 180));
    motor4.write(map(throttle, 0, 1023, 0, 180));

    // setAllMotorsSpeed(motorSpeed);

    // // sow data on LCD
    // lcd.print((int)motorSpeed);
    // lcd.print(',');
    // lcd.print((int)pitch);
    // lcd.print(',');
    // lcd.print((int)yaw);
    // lcd.print(',');
    // lcd.print((int)roll);
    break;
  }

  delay(10);
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

void setMotorSpeed(int motorID, float throttle)
{
  motor[motorID].write(map(throttle, -1, 1, 0, 180));
}

void setAllMotorsSpeed(float throttle)
{
  int i;
  for (i = 0; i < NUMBER_OF_MOTORS; i++)
  {
    motor[i].write(map(throttle, -1, 1, 0, 180));
  }
}

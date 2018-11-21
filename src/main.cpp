#include <Arduino.h>
#include <Wire.h>
#include <ZumoShield.h>
/* ––– Competition variables –––
  bool DEBUG = false;
  #define COLOR_EDGE = >

  Remember to change following:
  - Speed control -> 1
*/
bool DEBUG = true;                          // Debug: true for debugging
const int SPEED_CONTROL = 1;
#define COLOR_EDGE >                        //Quick shift variable for edge color. Black edge = '>' , white edge = '<'

// Sensor variables
const int ON_BOARD_LED = 13;
const int NUM_SENSORS = 6;                  // Number of light sensors on robot
unsigned int sensor_values[NUM_SENSORS];    // Array to store sensor
const int QTR_THRESHOLD = 1500;             // microseconds, threshold for accelerometer :

/*
  SPEEDS: define different speed levels
  0-400 : 400 Full speed

  SPEEDCONTROL:
  1 for normal speed
  2 for testing speed.
*/
const int FULL_SPEED =         400/SPEED_CONTROL;
const int FULL_REVERSE_SPEED = -400/SPEED_CONTROL;
const int REVERSE_SPEED =      -350/SPEED_CONTROL; // negative value
const int TURN_SPEED =         250/SPEED_CONTROL;
const int FORWARD_SPEED =      100/SPEED_CONTROL;
const int SEARCH_SPEED =       250/SPEED_CONTROL;

// Duration : Timing constants
const int REVERSE_DURATION =   500; // ms
const int TURN_DURATION =      350; // ms

// Timing for accelerometer
unsigned long loop_start_time;

ZumoBuzzer buzzer;
 // use V0 to suppress sound effect; v15 for max volume
ZumoMotors motors;
Pushbutton button(ZUMO_BUTTON); // pushbutton on pin 12
ZumoReflectanceSensorArray sensors(QTR_NO_EMITTER_PIN);


unsigned int nextTimeout = 0;

// STATES
const int S_STANDBY = 0;
const int S_FLIGHT = 1;
const int S_SCOUT = 2;

int state = S_STANDBY;

/*
  Pulls robot fast in reverse speed to get the plow down.
*/

void plowDown(){
  motors.setSpeeds(REVERSE_SPEED, REVERSE_SPEED);
  delay(100);
}

/*
  Waits for button to be pressed and initiate start sequence
*/

bool waitForButtonAndCountDown()
{
  button.waitForPress();
  digitalWrite(ON_BOARD_LED, HIGH);
  button.waitForRelease();
  digitalWrite(ON_BOARD_LED, LOW);

  // play audible countdown
  for (int i = 0; i < 3; i++)
  {
    delay(1000);
    buzzer.playNote(NOTE_G(3), 200, 15);
  }
  delay(1000);
  buzzer.playNote(NOTE_G(4), 500, 15);
  return true;
}

void setup()
{
  Wire.begin();

  // Initiate LSM303
  lsm303.init();
  lsm303.enable();
  // uncomment if necessary to correct motor directions
  //motors.flipLeftMotor(true);
  //motors.flipRightMotor(true);

  pinMode(ON_BOARD_LED, OUTPUT);

  Serial.begin(9600);

  // reset loop variables
  in_contact = false;  // 1 if contact made; 0 if no contact or contact lost
  contact_made_time = 0;
  last_turn_time = millis();  // prevents false contact detection on initial acceleration
  _forwardSpeed = SlowSpeed;
  full_speed_start_time = 0;
  //Waiting for button to be pressed to start loop
  waitForButtonAndCountDown();
  plowDown();   // call plowdown function to pull the plow down from upward position
}

void loop(){
  unsigned long startOfLoopTime;
  sensors.read(sensor_values);
  int valFromIRSensor = analogRead(A0);
  double distanceSensor = constrain(valFromIRSensor, 200, 800);

  if(DEBUG){
   startOfLoopTime = millis();
  }
  if (sensor_values[0] COLOR_EDGE QTR_THRESHOLD || sensor_values[5] COLOR_EDGE QTR_THRESHOLD) {
    // if leftmost sensor detects line, reverse and turn to the
    motors.setSpeeds(REVERSE_SPEED, REVERSE_SPEED);
    delay(REVERSE_DURATION);
  } else {
    if(distanceSensor > 220){
      motors.setSpeeds(FULL_SPEED,FULL_SPEED);
    } else {
      motors.setSpeeds(SEARCH_SPEED, -SEARCH_SPEED);
    }
  }

  if(DEBUG){
    unsigned long endOfLoopTime = millis();
    if(endOfLoopTime != startOfLoopTime){
      Serial.print("Loop run time: ");
      Serial.print(endOfLoopTime - startOfLoopTime);
      Serial.println(" ms");
    } else {
      Serial.println("Super fast program. Loopruntime is ZERO!");
    }
  }
}

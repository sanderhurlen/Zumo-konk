#include <Arduino.h>
#include <Wire.h>
#include <ZumoShield.h>

/* ––– Competition variables –––
  bool DEBUG = false;
  #define COLOR_EDGE = >

  Remember to change following:
  - Speed control -> 1
*/
bool DEBUG = false;
bool DEBUG_SENSOR = false;                         // Debug: true for debugging
const int SPEED_CONTROL = 1;
#define COLOR_EDGE <                        //Quick shift variable for edge color. Black edge = '>' , white edge = '<'

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
const int SEARCH_SPEED =       200/SPEED_CONTROL;

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
  Function to pull robot fast in reverse speed to get the plow down.
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
  if(DEBUG || DEBUG_SENSOR){
    Serial.begin(9600);
  }
  //Wire.begin();

  pinMode(ON_BOARD_LED, OUTPUT);

  //Waiting for button to be pressed to start loop
  waitForButtonAndCountDown();
  plowDown();   // call plowdown function to pull the plow down from upward position
}

void loop(){
  //Debug varialbe to time loop runtime
  unsigned long startOfLoopTime;

  //Stores reflectans sensor readings
  sensors.read(sensor_values);

  //Take readings from IR sensor
  int valFromIRSensor = analogRead(A0);

  //Constraining readings from IR to avoid spikes
  double distanceSensor = constrain(valFromIRSensor, 200, 800);

  if(DEBUG){
   startOfLoopTime = millis();
  }
  //Check if the bot is on border else go to search/attack stage
  if (sensor_values[0] COLOR_EDGE QTR_THRESHOLD || sensor_values[5] COLOR_EDGE QTR_THRESHOLD) {
    motors.setSpeeds(REVERSE_SPEED, REVERSE_SPEED);
    delay(REVERSE_DURATION);
  } else {
    //If target is within range give full speed ahead.
    if(distanceSensor > 204){
      motors.setSpeeds(FULL_SPEED,FULL_SPEED);
    //If no target within range, scan for target by spinning around
    } else {
      motors.setSpeeds(SEARCH_SPEED, -SEARCH_SPEED);
    }
  }

  if(DEBUG_SENSOR){
    Serial.println(distanceSensor);
    delay(100);
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

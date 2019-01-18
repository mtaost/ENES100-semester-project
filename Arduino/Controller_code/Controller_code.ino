#include <NewPing.h>
#include "enes100.h"
/* NOTES
 *  Chemical spill location: (2180, 1320)
 */
//Pool positions
#define POOL_X 2.19
#define POOL_Y 1.415

//RF module communications
enes100::RfClient<HardwareSerial> rf(&Serial1);
enes100::Marker marker;

//Sensor pin allocation
#define TRIG_RIGHT 13
#define ECHO_RIGHT 12

#define TRIG_LEFT 16
#define ECHO_LEFT 16
NewPing left(TRIG_LEFT, ECHO_LEFT, 100);
NewPing right(TRIG_RIGHT, ECHO_RIGHT, 100);

//H-Bridge allocations
#define enA 11
#define in1 9
#define in2 8
#define in3 3
#define in4 2
#define enB 10

//On board motor pins
#define ROMEO_E2 6
#define ROMEO_M2 7
#define ROMEO_E1 5
#define ROMEO_M1 4

//pH meter stuff
#define SensorPin A0         //pH meter Analog output to Arduino Analog Input 0
#define Offset 0.06          //deviation compensate

//Relay channels
#define relay_CH1 15
#define relay_CH2 14

//Our Marker ID
#define MARKER_ID 18

//Boolean to describe platform state
boolean platformUp = true;

//Maximum PWM speed for motors
int MAX_SPEED = 255;

//Sort of PID
float thetaDes = 0.0;
float thetaError;

//More constants
int osvSpeed = 100;

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  delay(500);
  pinMode(ROMEO_E2, OUTPUT);
  pinMode(ROMEO_M2, OUTPUT);
  pinMode(ROMEO_M1, OUTPUT);
  pinMode(ROMEO_E1, OUTPUT);
  
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  pinMode(relay_CH1, OUTPUT); //For the solenoid
  pinMode(relay_CH2, OUTPUT); //For the stopper remover
  beep(5); //Startup tone
  rf.resetServer();
  rf.sendMessage("\nTeam STS hath arrived.\n");
}

void loop() {
  rf.sendMessage("\nWaiting for marker...\n");
  getMarker();
  rf.sendMessage("\nGot marker!\n");
  moveTo(.50, 1.0); //Move to center of the starting area
  //Code here to determine which path along x axis is clear
  moveTo(1.0, marker.y); //Move to midway x position
  moveTo(POOL_X, POOL_Y); //Move to the pool
  
  //Nudge forward a bit
  move(85);
  delay(100);
  stop();

  baseObjective();
  rf.sendMessage("\nTaking sample\n");
  triggerSolenoid();
  rf.sendMessage("\nWaiting for sample\n");
  for(int i = 0; i <= 20; i++) {
    rf.sendMessage(20 - i);
    delay(1000);
  }
  rf.sendMessage("\nNeutralizing\n");
  stopperMotor();
  pump(true);
  for(int i = 0; i <= 180; i++) {
    rf.sendMessage("\nCurrent pH: \n");
    rf.sendMessage(pH());
    rf.sendMessage("\nTime left: \n");
    rf.sendMessage(180 - i);
    delay(1000);
  }
  rf.sendMessage("\nComplete!\n");
  pump(false);
  raisePlatform();
  while(true){delay(1000);} //Stop the presses!
}

//Moves from current position to a given x, y coordinate
void moveTo(float x, float y) {
  rf.sendMessage("\nAttempting to move to: \n");
  rf.sendMessage(x);
  rf.sendMessage(" , ");
  rf.sendMessage(y);
  int counter = 0;
  while (!atTarget(x, y)) {
    if (counter % 5 == 0) //Does a theta check every so often
      turnToFace(x, y); //Turns to the correct angle
    rf.sendMessage("\n ====MOVING==== \n");
    move(100); //Otherwise move at full speed
    delay(200);
  }
  stop();
  delay(200);
}

//Accepts speeds from -100 to 100
void moveLeft(int speed) {
  speed = map(speed, 0, 100, 0, MAX_SPEED);
  if (speed > 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  if (speed < 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  speed = abs(speed);
  analogWrite(enA, speed);
}

void moveRight(int speed) {
  speed = map(speed, 0, 100, 0, MAX_SPEED);
  if (speed > 0) {
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  }
  if (speed < 0) {
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  }
  speed = abs(speed);
  analogWrite(enB, speed);
}

void move(int speed) {
  moveRight(speed);
  moveLeft(speed);
}

void stop() {
  move(0);
}

//Converts their theta to a normal theta
float convertTheta(float t) {
  if (t < 0.0) {
    return 6.283 + t;
  }
  return t;
}

//Turns to a theta t
void turnToFace(float x, float y) {
  float td = thetaDesired(x, y);
  if (rf.receiveMarker(&marker, MARKER_ID)) {
    while (fabs(td - convertTheta(marker.theta)) > .2) {
      rf.receiveMarker(&marker, MARKER_ID);
      rf.sendMessage("\nTrying to turn to: \n");
      rf.sendMessage(td);
      rf.sendMessage("\nCurrently at: \n");
      rf.sendMessage(convertTheta(marker.theta));
      float tc = convertTheta(marker.theta);
      float theta;
      float phi;
      if (tc  < td) {
        theta = 6.28 - (td - tc);
        phi = td-tc;
        if (theta < phi) {
          moveLeft(95);
          moveRight(-95);
        }
        if (theta > phi) {
          moveLeft(-95);
          moveRight(95);
        }
      }
      if (tc > td) {
        theta = 6.28 - (tc - td);
        phi = tc - td;
        if (theta < phi) {
          moveLeft(-95);
          moveRight(95);
        }
        if (theta > phi) {
          moveLeft(95);
          moveRight(-95);
        }
      }
      
      td = thetaDesired(x, y); //Update heading
      delay(100);
    }
    stop();
  }
}

//Returns theta desired to get to destination
float thetaDesired(float x, float y) {
  if (getMarker()) {
    float deltaX = x - marker.x;
    float deltaY = y - marker.y; 
    float angle = atan2(deltaY, deltaX);
    if(angle < 0.0)
      return angle + 6.28;
    else {return atan2(deltaY, deltaX);}
  }
}

//Sees if currently close enough to target
boolean atTarget(float x, float y) {
  if (rf.receiveMarker(&marker, MARKER_ID)) {
    float deltaX = fabs(marker.x - x);
    float deltaY = fabs(marker.y - y);
    
    if (deltaX <= .1 && deltaY <= .1)
      return true;
  }
  return false;
}

//MUST BE USED WHEN MARKER IS UPDATED
//Finds distance from current to place
float distanceTo(float x, float y) {
  return sqrt(square(marker.x - x) + square(marker.y - y));
}

void triggerSolenoid() {
  digitalWrite(relay_CH1, HIGH);
  delay(1000);
  digitalWrite(relay_CH1, LOW);
  delay(500);
}

void stopperMotor() {
  digitalWrite(relay_CH2, HIGH);
  delay(2500);
  digitalWrite(relay_CH2, LOW);
  delay(1000);
  pinMode(relay_CH1, INPUT);
}

void baseObjective() {
  Serial.println("Attempting base objective");
  rf.sendMessage("\nAttempting base objective\n");
  beep(3);
  lowerPlatform();
  rf.sendMessage("\nWaiting for pH...\n");
  pump(true);
  delay(10000);
  pump(false);
  rf.sendMessage("\npH: \n");
  rf.sendMessage(pH());
  delay(3000);
}

//Waits for a marker
boolean getMarker() {
  while(!rf.receiveMarker(&marker, MARKER_ID)) {delay(100);}
  return true;
}

//Runs neutralization thing for a while
void neutralize() {
  int timeout = 300; //5 minute timeout (300 seconds)
  pump(true); //Turn on the pump
  stopperMotor(); //Dispense the base
  while (pH() < 6.0) {
    timeout++;
    delay(1000);
  }
  pump(false);
  rf.sendMessage("\nnew pH: \n");
  rf.sendMessage(pH());
}

void lowerPlatform() {
  //Only allows lowering if it hasn't lowered
  if (platformUp) {
    Serial.println("Lowering Platform");
    analogWrite(ROMEO_E2, 75);
    digitalWrite(ROMEO_M2, HIGH);
    delay(2200);
    analogWrite(ROMEO_E2, 0);
  }
  platformUp = false;
}

void raisePlatform() {
  if (!platformUp) {
    Serial.println("Raising Platform");
    analogWrite(ROMEO_E2, 100);
    digitalWrite(ROMEO_M2, LOW);
    delay(2000);
    analogWrite(ROMEO_E2, 80);
    delay(1500);
    analogWrite(ROMEO_E2, 0);
  }
  platformUp = true;
}

//Handles the pump
void pump(boolean engaged) {
  if (engaged) {
    analogWrite(ROMEO_E1, 255);
    digitalWrite(ROMEO_M1, HIGH);
  }
  if (!engaged) {
    analogWrite(ROMEO_E1, 0);
    digitalWrite(ROMEO_M1, HIGH);
  }
}

//Returns distance from right ultrasonic sensor
int rightDist() {
  return right.ping_cm();
}

//Returns distance from left ultrasonic sensor
int leftDist() {
  return left.ping_cm();
}

void printDistances() {
  delay(100);
  Serial.print("Right: ");
  Serial.print(rightDist());
  Serial.print(" cm | ");
  Serial.print("Left: ");
  Serial.print(leftDist());
  Serial.println(" cm");
}

//Returns the measured pH value
float pH() {
  float pHValue,voltage;
  voltage = analogRead(SensorPin) * 5.0/1024.0;
  pHValue = 3.5*voltage+Offset;
  return pHValue;
}

void printPosition() {
  if (rf.receiveMarker(&marker, MARKER_ID)){
    Serial.print("theta: ");
    Serial.println(marker.theta);
    Serial.print("x : ");
    Serial.println(marker.x, 3);
    Serial.print("y: ");
    Serial.println(marker.y, 3);
    Serial.println("");
  }
}

//Plays a number of tones through the motors
void beep(int n) {
  for (int i = 0; i < n; i++) {
    move(25);
    delay(50);
    stop();
    delay(50);
  }
}

//Plays a long and then a short tone
void beep1() {
  move(20);
  delay(500);
  stop();
  delay(100);
  move(20);
  delay(100);
  stop();
}



#include <Arduino.h>
#include <Servo.h>
#include "Wire.h"
#include "new_pins.h"
#include "constants.h"
#include "veml6040.h"

int sensors[8]     = {0};
int firstLineIndex = -1;
int lastLineIndex  = -1;
int targetIndex    =  3;
int amountSeen     = 0;

Servo rightArm;
Servo leftArm;
Servo sorter;
Servo leftDump;
Servo rightDump;

void setup() {

  // initialize line sensors
  for(int i = 0; i < 8; i++)
  {
    pinMode(LINE_SENSOR[i], INPUT);
  }

  // initialize motor controllers
  pinMode(WHEEL_DIR_LB, OUTPUT);
  pinMode(WHEEL_DIR_LF, OUTPUT);
  pinMode(WHEEL_DIR_RB, OUTPUT);
  pinMode(WHEEL_DIR_RF, OUTPUT);

  pinMode(LEDY, OUTPUT);
  pinMode(LEDG, OUTPUT);

  pinMode(WHEEL_SPEED_L, OUTPUT);
  pinMode(WHEEL_SPEED_R, OUTPUT);

  pinMode(WHEEL_STBY, OUTPUT);

  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(BACK_LEFT_SENSOR, OUTPUT);
  pinMode(L_DUMP,INPUT);

  leftArm.attach(L_TUBE);
  sorter.attach(SORTER);
  sorter.write(PICK_UP);
  leftArm.write(100);
  leftDump.attach(L_DUMP);
  leftDump.write(DONT_DUMP_POS);//initalize servo at perfect position IMH.

  writeWheelDirection(WHEEL_FORWARDS, WHEEL_FORWARDS);
  digitalWrite(WHEEL_STBY  , HIGH);

  Serial3.begin(115200);
  Serial.begin(115200);
  Serial3.println("Starting Up...");
}

/*
                            MANUEVERING FUNCTIONS:

  The following functions relate to maneuvering the robot around the track, and
  resulting states for accomplishing that task.

*/

/*
  Reads in the line sensor's current state, and stores them in the global
  variables above.
*/
void readLine() {
  amountSeen    = 0;
  lastLineIndex = -1;
  for(int i = 7; i >= 0; --i) {
    sensors[i] = digitalRead(LINE_SENSOR[i]);
    if(sensors[i] == HIGH) {
      if(lastLineIndex == -1) {
        lastLineIndex = i;
      }
      ++amountSeen;
      firstLineIndex = i;
    }
  }
}

bool twoConsecutive() {
  int lowCount = 0;
  bool consecutive = false;
  for(int i = 0; i < 7; i++) {
    if(sensors[i] == HIGH && sensors[i + 1] == HIGH) {
      consecutive = true;
    }

    if(sensors[i] == LOW) {
      lowCount++;
    }
  }
  if(sensors[7] == LOW) {
    lowCount++;
  }
  return lowCount == 6 && consecutive;
}

void writeWheelDirection(bool ldir, bool rdir) {
  digitalWrite(WHEEL_DIR_LF, ldir);
  digitalWrite(WHEEL_DIR_LB, !ldir);
  digitalWrite(WHEEL_DIR_RF, rdir);
  digitalWrite(WHEEL_DIR_RB, !rdir);
}

/*
  "Beautiful absolute value function [stolen] from stack overflow. Behold it!"
    - Daschel Fortner
*/
int absVal(int val) {
  return val * ((val > 0) - (val < 0));
}

/*
  Write a direction to the wheels. If the wheel speed passed is negative, then
  the direction of that wheel is switched. Don't pass something over 255 or
  under 0.
*/
void writeToWheels(int ls, int rs) {
  writeWheelDirection(ls > 0, rs > 0); // Clean? Maybe more confusing?
  if(rs > 0) {
    digitalWrite(LEDG, HIGH);
  } else {
    digitalWrite(LEDG, LOW);
  }
  if(ls > 0) {
    digitalWrite(LEDY, HIGH);
  } else {
    digitalWrite(LEDY, LOW);
  }
  analogWrite(WHEEL_SPEED_L, absVal(ls));
  analogWrite(WHEEL_SPEED_R, absVal(rs));
}

/*
  Follows the line, returns true if the robot is at a fork.
*/
bool lineFollow(int ts, int strictness) {
  int offset = firstLineIndex - TARGET_INDEX;
  int rightSpeed = ts - offset * strictness;
  int leftSpeed = ts + offset * strictness;
  writeToWheels(leftSpeed % 255, rightSpeed % 255);

  // Return true if the sensors can see a fork
  return amountSeen > TURN_AMOUNT;
}

/*
  Turns based on direction. Use the constants LEFT and RIGHT for this function.
  Returns true if the robot is back on the line.
*/
bool turn(int spd, char dir) {
  int targetIndex = TARGET_INDEX;
  if(dir == LEFT){
    writeToWheels(-spd, spd);
    targetIndex += 3;
  }else{
    targetIndex -= 3;
    writeToWheels(spd, -spd);
  }
  // Return true if the robot is back centered on the line
  return twoConsecutive();
  //return atMiddle();
  //return firstLineIndex >= targetIndex  && amountSeen < 3;
}

/*
  Delays for ms number of milliseconds.
*/
bool delayState(int ms) {
  static int milliseconds = -1;
  if(milliseconds == -1) {
    milliseconds = millis();
  }
  else if(millis() - milliseconds >= ms) {
    milliseconds = -1;
    return true;
  }
  return false;
}

/*
  Switches the wheels backwards and then does a big, swooping turn.
*/
bool swoopTurn(char dir, int ts, int d) {
  if(dir == LEFT) {
    //lift up arms.
    writeToWheels(0, ts);
  } else {
    writeToWheels(ts, 0);
  }

  return delayState(d);
}

bool backToCornerState(int ts, int strictness)
{
  //writeToWheels(ts, ts);
  int lSens, rSens;


  // Get info from wall sensor and update lSens and rSens
  lSens = readBackLeft();
  rSens = readBackRight();

  /*int offset = (lSens - rSens) / strictness;
  //offset = 10;*/
  writeToWheels(-SLOW_SPEED, -SLOW_SPEED);


  return rSens <= 65; // This is because we don't know what the sensors will return (they aren't always the same)
  // FIXME The return needs to be tested

  //return delayState(d);
}

bool findLine(int ts)
{
  writeToWheels(ts, ts);
  return amountSeen >= 2;
}

bool waitState() {
  writeToWheels(0, 0);
  return (digitalRead(BUTTON1) == LOW);
}

bool doTurnSequence(const char sequence[], int index) {
  static bool turning = false;
  sortBalls(turning);
  otherPrintVar = turning;
  if(turning) {
    if(turn(100, sequence[index])) {
      turning = false;
      return true;
    }
  } else {
    turning = lineFollow(100, 50);
  }
  return false;
}

// This will take care of the entire run starting from the end of ball pick-up
// all the way through first dump
bool firstCornerState(char dir) {
  //TODO
}

// This will take care of the entire run starting from the end of firstCornerState
// all the way through second dump
bool secondCornerState(char dir) {
  //TODO
}

int readBackRight(){//gets data val from right infraread sensor IMH
 return analogRead(BACK_RIGHT_SENSOR);
}

int readBackLeft(){//gets data val from left infraread sensor IMH
  return analogRead(BACK_LEFT_SENSOR);
}

/*
                               COLOR SORTING CODE:

  The following functions all relate to color sorting.
*/

bool sort(int color) {
  //TODO
}

void sortBalls(bool turning) {
  //TODO
}

void loop() {
  //TODO
}

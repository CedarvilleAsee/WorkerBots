#include <Arduino.h>
#include <Servo.h>
#include "new_pins.h"
#include "constants.h"
#include "veml6040.h"

int sensors[8]     = {0};
int firstLineIndex = -1;
int lastLineIndex  = -1;
int targetIndex    =  3;
int amountSeen     = 0;
bool running       = false;
bool on            = false;
int printVar = 0;
bool otherPrintVar = false; // Consider making "printVar" an array

int subState   = 0;
int iterations = 0;

Servo rightArm;
Servo leftArm;
Servo sorter;

// Color Sensor object
VEML6040 colorSensor;

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

  leftArm.attach(L_TUBE);
  sorter.attach(SORTER);

  // Wall Sensors
  //pinMode(L_BARREL_SENSOR, INPUT);
  //pinMode(BACK_SENSOR, INPUT);

  writeWheelDirection(WHEEL_FORWARDS, WHEEL_FORWARDS);
  digitalWrite(WHEEL_STBY  , HIGH);

  // Initialize the color sensor
  colorSensor.setConfiguration(VEML6040_IT_320MS + VEML6040_AF_AUTO 
                                   + VEML6040_SD_ENABLE);

  if(!colorSensor.begin()) {
    digitalWrite(LEDY, HIGH);
  }
  Serial.begin(115200);
  Serial3.begin(115200);
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

/*
  New and improved wheel direction switch-er-oo. Use the constants FORWARDS and
  and BACKWARDS for this function. You could use true and false, but that would
  be very ape-ish of you. 
*/
void writeWheelDirection(bool ldir, bool rdir) {
  digitalWrite(WHEEL_DIR_LF, ldir);
  digitalWrite(WHEEL_DIR_LB, !ldir);
  digitalWrite(WHEEL_DIR_RF, rdir);
  digitalWrite(WHEEL_DIR_RB, !rdir);
}

void old_writeWheelDirection(bool ldir, bool rdir) {
  if(ldir) {
    digitalWrite(WHEEL_DIR_LB, LOW );
    digitalWrite(WHEEL_DIR_LF, HIGH);
  } else {
    digitalWrite(WHEEL_DIR_LB, HIGH);
    digitalWrite(WHEEL_DIR_LF, LOW);
  }

  if(rdir) {
    digitalWrite(WHEEL_DIR_RF, HIGH);
    digitalWrite(WHEEL_DIR_RB, LOW );
  } else {
    digitalWrite(WHEEL_DIR_RF, LOW);
    digitalWrite(WHEEL_DIR_RB, HIGH);
  }
}

/*
  Beautiful absolute value function retreived from stack overflow. Behold it!
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
  absVal(ls);
  absVal(rs);
  writeWheelDirection(ls > 0, rs > 0); // Clean? Maybe more confusing?
  analogWrite(WHEEL_SPEED_L, ls);
  analogWrite(WHEEL_SPEED_R, rs);
}

/*
  Follows the line, returns true if the robot is at a fork.
*/
bool lineFollow(int ts, int strictness) {
  int offset = firstLineIndex - TARGET_INDEX;
  int rightSpeed = ts - offset * strictness;
  int leftSpeed = ts + offset * strictness;
  writeToWheels(leftSpeed, rightSpeed);

  // Return true if the sensors can see a fork
  return amountSeen > TURN_AMOUNT;
}

/*
  Turns based on direction. Use the constants LEFT and RIGHT for this function.
  Returns true if the robot is back on the line.
*/
bool turn(int speed, char direction) {
  if(direction == LEFT){
    writeToWheels(-speed, speed);
  }else{
    writeToWheels(speed, -speed);
  }
  // Return true if the robot is back centered on the line
  return firstLineIndex >= TARGET_INDEX  && amountSeen < 3;
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
  writeWheelDirection(BACKWARDS, BACKWARDS);
  if(dir == LEFT) {
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
  //lSens = analogRead(L_BARREL_SENSOR);
  //rSens = analogRead(BACK_SENSOR);
  
  int offset = (lSens - rSens) / strictness;
  //offset = 10;
  writeToWheels(ts + offset, ts - offset);

  return lSens <= 300 && rSens <= 300; // This is because we don't know what the sensors will return (they aren't always the same)
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
  otherPrintVar = turning;
  if(turning) {
    if(turn(150, sequence[index])) {
      turning = false;
      return true;
    }
  } else {
    turning = lineFollow(100, 26);
  }
  return false;
}

bool followTrackState() {
  static int state = 0;

  printVar = state;
  bool isFinished = false;
  
  switch(state) {
    case 3:
      rightArm.write(50);
      break;
    case 10:
      leftArm.write(145);
      break;
    case 14:
      isFinished = amountSeen > TURN_AMOUNT;
      break;
    default:
      rightArm.write(100);
      leftArm.write(95);
      break;
  }

  if(doTurnSequence(TURN_SEQUENCE, state)) 
    state++;
  return isFinished;
}

bool cornerState(char dir) {
  static int state = 0;
  printVar = state;
  switch(state) 
  {
    case 0:
      if(swoopTurn(dir, 255, 2150))  state++; 
      break;
    case 1:
      if(backToCornerState(100, 10)) { 
        state++;
        writeWheelDirection(WHEEL_FORWARDS, WHEEL_FORWARDS);
      }
      break;
    case 2:
      writeToWheels(0, 0);
      if(delayState(2000) && dir == RIGHT) {
        state++; 
      }
      else if(dir == LEFT) {
        state = 0;
        return true;
      }
      break;
    case 3:
      if(findLine(150)) {
        state = 0;
        return true;
      }
      break;
  }
  return false;
}

bool goToNextCornerState() {
  static int state = 0; 
  printVar = state;
  bool isFinished = false;

  if(state == 4) {
    isFinished = amountSeen > TURN_AMOUNT;
  }
  if(doTurnSequence(SECOND_TURN_SEQUENCE, state)) state++;

  return isFinished;
}

bool secondCornerState() {

  static int state = 0;
  printVar = state;
  switch(state) 
  {
    case 0:
      if(swoopTurn(LEFT, 255, 2150))  state++; 
      break;
    case 1:
      if(backToCornerState(100, 10)) { 
        state++;
        writeWheelDirection(WHEEL_FORWARDS, WHEEL_FORWARDS);
      }
      break;
    case 2:
      writeToWheels(0, 0);
      if(delayState(2000))  state++; 
      break;
    case 3:
      if(findLine(150)) {
        state = 0;
        return true;
      }
      break;
  }
  return false;
}

/*
                               COLOR SORTING CODE:

  The following functions all relate to color sorting.

*/
bool isBallPresent() {
  return colorSensor.getWhite() > BALL_TRIGGER;
}

int getBallData() {
  if(colorSensor.getWhite() > BALL_TRIGGER) {
    //return (colorSensor.getBlue() > BLUE_TRIGGER) ? BLUE_BALL : ORANGE_BALL;
    return colorSensor.getBlue();
  } else {
    return NO_BALL;
  }
}

bool sort(int color) {
  sorter.write(color);
  return delayState(1000);
}

void sortBalls() {
  static bool sorting = false;
  if(sorting) {
    sorting = sort(getBallData());
  } else {
    sorting = isBallPresent();
  }
}

/*
                              TEST FUNCTIONS:

  The following functions all relate to testing various parts of the robot.

*/

/*
  Tests a wheel based on a target speed and the constants LEFT and RIGHT.
*/
void testWheel(char wheel, int ts)
{
  if(wheel == LEFT)
    writeToWheels(ts, 0);
  else
    writeToWheels(0, ts);
}

void loop() {
  static int state = -1;
  static int sorterTester = 0;
  readLine();
  switch(state)
  {
    case 0:
      if(waitState())  state++; 
      break;
    case 1:
      if(followTrackState())  state++; 
      break;
    case 2:
      if(cornerState(RIGHT))  state = 0;
      break;
    case 3:
      if(goToNextCornerState()) state++;
      break;
    case 4:
      if(cornerState(LEFT)) state = 0;
    default:
      if(true) {
        sorter.write(70);
      } else {
        sorter.write(125);
      }
      break;
  }

  iterations++;
  if(iterations == BLUETOOTH_LIMITER)
  {
    iterations = 0;
    //Serial3.print("[ ");
    //for(int i = 0; i < 8; i++)
    //{
    //  Serial3.print(sensors[i]);
    //  Serial3.print(" ");
    //}
    //Serial3.println("]");
    //Serial3.print("[ ");
    //Serial3.print(firstLineIndex);
    //Serial3.print(", ");
    //Serial3.print(amountSeen);
    //Serial3.print(" ]");
    //Serial3.print(" - ");
    //Serial3.print(printVar);
    //Serial3.print(", ");
    //Serial3.println(otherPrintVar);

    /*Serial3.print("Get Data = ");
    Serial3.print(getBallData());
    Serial3.print(" White val: ");
    Serial3.println(colorSensor.getBlue());*/


  float b = colorSensor.getBlue();
  float w = colorSensor.getWhite();
  float g = colorSensor.getGreen();
  float r = colorSensor.getRed();
  Serial3.print("Blue: ");
  Serial3.print(b / w);
  Serial3.print(b);
  Serial3.print(" Red: ");
  Serial3.println(r / w);
  Serial3.print(r);
  Serial3.print(" Green: ");
  Serial3.println(g / w);
  Serial3.print(g);
  }
}

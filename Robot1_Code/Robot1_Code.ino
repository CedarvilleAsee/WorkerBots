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
int printVar = 0;
bool otherPrintVar = false; // Consider making "printVar" an array

Servo rightArm;
Servo leftArm;
Servo sorter;
Servo leftDump;
Servo rightDump;

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
  pinMode(BACK_LEFT_SENSOR, OUTPUT);
  pinMode(L_DUMP,INPUT);

  leftArm.attach(L_TUBE);
  rightArm.attach(R_TUBE);

  sorter.attach(SORTER);
  sorter.write(PICK_UP);

  // arms should be written to x_ARM_UP, and then put down when we actually
  // start
  leftArm.write(L_ARM_UP);
  rightArm.write(R_ARM_UP);

  leftDump.attach(L_DUMP);
  leftDump.write(L_DONT_DUMP);//initalize servo at perfect position IMH.

  rightDump.attach(R_DUMP);
  rightDump.write(R_DONT_DUMP);//initalize servo at perfect position IMH.

  // Wall Sensors
  //pinMode(L_BARREL_SENSOR, INPUT);
  //pinMode(BACK_SENSOR, INPUT);

  writeWheelDirection(WHEEL_FORWARDS, WHEEL_FORWARDS);
  digitalWrite(WHEEL_STBY  , HIGH);

  Serial.begin(9600);
  Serial3.begin(115200);
  Serial3.println("Starting Up...");

  // initialize color sensor
  //Serial.begin(9600);
  Wire.begin();
  delay(500);
  // MAKE SURE TO CHECK FOR NO YELLOW LED!
  if(!colorSensor.begin()) {
    digitalWrite(LEDY, HIGH);
  }
  colorSensor.setConfiguration(VEML6040_IT_320MS + VEML6040_AF_AUTO + VEML6040_SD_ENABLE);
  delay(1500);

}

/*
MANUEVERING:

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
  firstLineIndex = -1;
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

bool twoConsecutiveAtMiddle() {
  return twoConsecutive() && firstLineIndex >= TARGET_INDEX;
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

  if(amountSeen == 0) {
    writeToWheels(HALF_SPEED, HALF_SPEED);
  } else {
    writeToWheels(leftSpeed % 255, rightSpeed % 255);
  }

  // Return true if the sensors can see a fork
  return amountSeen > TURN_AMOUNT;
}

bool funnelState(int ts, int strictness) {
  if(firstLineIndex == -1 && lastLineIndex == -1) {
    writeToWheels(HALF_SPEED, HALF_SPEED);
  } else {
    if(firstLineIndex <= TARGET_INDEX) {
      writeToWheels(HALF_SPEED, HALF_SPEED - (strictness * firstLineIndex));
    } else {
      writeToWheels(HALF_SPEED - (strictness * (8 - firstLineIndex)), HALF_SPEED);
    }
  }

  return amountSeen > TURN_AMOUNT;
}

/*
   Turns based on direction. Use the constants LEFT and RIGHT for this function.
   Returns true if the robot is back on the line.
 */
bool turn(int spd, char dir) {
  if(dir == LEFT){
    writeToWheels(-spd, spd);
  }else{
    writeToWheels(spd, -spd);
  }
  // Return true if the robot is back centered on the line
  return twoConsecutive();
  //return atMiddle();
  //return firstLineIndex >= targetIndex  && amountSeen < 3;
}

bool correctJank(int spd, int strictness) {
  writeToWheels(spd - strictness, spd);
  return twoConsecutiveAtMiddle();
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
    writeToWheels((0 - ts) % 5, ts);
  } else {
    writeToWheels(ts, (0 - ts) % 20);
  }

  return delayState(d);
}

bool backToCornerState(int ts, int strictness)
{
  //writeToWheels(ts, ts);
  int lSens, rSens;
  int offset;

  // Get info from wall sensor and update lSens and rSens
  lSens = readBackLeft();
  rSens = readBackRight();

  if(lSens < 900 && rSens < 900) {
    offset = (lSens - rSens) / strictness;

    if(absVal(offset) > 255 - ts) {
      int sign = offset / absVal(offset);
      offset = (255 - ts) * sign;
    }
  } else {
    offset = 0;
  }

  writeToWheels(-(ts + offset), -(ts - offset));
  return rSens <= 60 && lSens <= 60; // This is because we don't know what the sensors will return (they aren't always the same)
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

/*
   Proven to work by cases.
 */
bool doTurnSequence(const char sequence[], int index, bool left, bool right) {
  static bool turning = false;
  if(turning) {
    if(left) {
      leftArm.write(L_ARM_HALF);
    } 

    if(right) {
      rightArm.write(R_ARM_HALF);
    }

    if(turn(HALF_SPEED, sequence[index])) {
      turning = false;
      return true;
    }
  } else {
    turning = lineFollow(HALF_SPEED, 50);
  }
  return false;
}

bool doTurnSequence(const char sequence[], int index) {
  static bool turning = false;
  if(turning) {
    if(turn(HALF_SPEED, sequence[index])) {
      turning = false;
      return true;
    }
  } else {
    turning = lineFollow(HALF_SPEED, 50);
  }
  return false;
}



/*
   Inductively proven to work.
 */
bool followTrackState() {
  static int state = 0;
  printVar = state;
  bool isFinished = false;

  switch(state) {
    case 0:
    case 1:
      rightArm.write(R_ARM_HALF);
      leftArm.write(L_ARM_DOWN);
      break;
    case 5:
      if(doTurnSequence(TURN_SEQUENCE, state, false, true)) {
        state++;
      }
      break;
    case 6:
      rightArm.write(R_ARM_HALF);
      break;
    case 11:
    case 12:
      leftArm.write(L_ARM_HALF);
      break;
    case 16:
      isFinished = amountSeen > TURN_AMOUNT;
      break;
    default:
      rightArm.write(R_ARM_DOWN);
      leftArm.write(L_ARM_DOWN);
      break;
  }

  if(state != 5) {
    if(doTurnSequence(TURN_SEQUENCE, state)) {
      state++;
    }
  }   

  sortBalls();

  return isFinished;
}

/*
   HEY ISAIAH!
   This is the big thing you need to work on. Here is where most of the work
   needs done.
 */
bool cornerState(char dir) {
  static int state = 0;
  printVar = state;
  switch(state)
  {
    case 0:
      leftArm.write(L_ARM_HALF);
      rightArm.write(R_ARM_HALF);
      if(swoopTurn(dir, -255, 2250))  state ++;
      break;
    case 1:
      if(backToCornerState(SLOW_SPEED, 20)) {
        state++;

        //writeWheelDirection(WHEEL_FORWARDS, WHEEL_FORWARDS);
      }
      break;
    case 2:
      writeToWheels(0, 0);
      // Add the drop off
      if(dir == RIGHT) {
        leftDump.write(L_DO_DUMP);//added 90 to initial position
      } else {
        rightDump.write(R_DO_DUMP);
      }
      if(delayState(2000) && dir == RIGHT) {
        state++;
      }
      else if(dir == LEFT) {
        state = 0;
        return true; // add celebration state????
      }
      break;
    case 3:
      if(turnFromCorner(530)) {
        state++;
      }
      break;
    case 4:
      /*
         In this state, we want to swing the robot to the right. It needs to
         turn to the right so that we can get it to the straight line beside
         the corner. We exit this state after a couple of milliseconds of delay.
         Once this state is over, the robot should be pointed towards the line
         and ready to go onward to the line.
       */
      if(dir == LEFT) {
        rightDump.write(R_DONT_DUMP);
      } else {
        leftDump.write(L_DONT_DUMP);
      }

      if(funnelState(HALF_SPEED, 40)) { //TODO: tweak delay value
        state++;
        // state = 0;
        // ^- if you want to test just this state, use state = 0, not state++
      }
      break;
    case 5: // In this state, we work to make it past the fork so we're ready
            // to do the turn sequence
      writeToWheels(HALF_SPEED, HALF_SPEED);
      if(twoConsecutive()) {
        state = 0;
        return true;
      }
      break;
    default:
      writeToWheels(0, 0);
      break;
  }
  return false;
}

bool goToNextCornerState() {
  static int state = 0;
  printVar = state;

  if(doTurnSequence(SECOND_TURN_SEQUENCE, state))
    state++;

  return state == 2 && amountSeen > TURN_AMOUNT;
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
        state=-1;
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

bool turnFromCorner(int d) {
  writeToWheels(150,0);
  return delayState(d);
}

/*
   COLOR SORTING:

   The following functions all relate to color sorting.

 */

bool isBallPresent() {
  return colorSensor.getWhite() > BALL_TRIGGER;
}

// If the red reading is greater than the green reading, then move to orange bin.
// If not, move the other way.
int getPositionFromBall() {
  if(colorSensor.getRed() > colorSensor.getGreen()){
    return ORANGE;
  } else {
    return WHITE;
  }
}

bool sort(int color) {
  sorter.write(color);
  return delayState(SORT_TIME);
}

// We sort only when a ball is present.
void sortBalls() {
  static bool sorting = false;
  static bool leftWiggle = true;
  static int  wiggleStart = millis();
  const  int  WIGGLE = 6;
  if(sorting) {
    sorting = !sort(getPositionFromBall());
  } else {

    // If it's time to move left
    if(leftWiggle) {
      // move left
      sorter.write(PICK_UP + WIGGLE);
    } else {
      // move right
      sorter.write(PICK_UP - WIGGLE);
    }

    // If we're done in one direction, move to the next
    if(millis() - wiggleStart >= 250) {
      leftWiggle = !leftWiggle;
      wiggleStart = millis();
    }

    sorting = isBallPresent(); // sorting = turning;
  }
}

/*
TESTING:

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
int readBackRight(){//gets data val from right infraread sensor IMH
  return analogRead(BACK_RIGHT_SENSOR);
}
int readBackLeft(){//gets data val from left infraread sensor IMH
  return analogRead(BACK_LEFT_SENSOR);
}

void loop() {
  static int state = 0;
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
      if(cornerState(RIGHT))  state++;
      break;
    case 3:
      if(goToNextCornerState()) state++;
      break;
    case 4:
      if(cornerState(LEFT)) state = 0;
    default:
      backToCornerState(SLOW_SPEED, 5);
      break;
  }

  static int serialLimiter = millis();
  if(millis() - serialLimiter >= 1000) {
    Serial.print("Right Sensor: ");
    Serial.println(readBackRight());
    Serial.print("Left Sensor: ");
    Serial.println(readBackLeft());
    Serial.println("---------------------------");
    serialLimiter = millis();
  }

}

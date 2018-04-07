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

int iterations = 0;

Servo rightArm;
Servo leftArm;
Servo sorter;
Servo leftDump;
Servo rightDump;

// Color Sensor object
//VEML6040 colorSensor;

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
  rightDump.attach(R_DUMP);
  rightDump.write(DONT_DUMP_POS);//initalize servo at perfect position IMH.

  // Wall Sensors
  //pinMode(L_BARREL_SENSOR, INPUT);
  //pinMode(BACK_SENSOR, INPUT);

  writeWheelDirection(WHEEL_FORWARDS, WHEEL_FORWARDS);
  digitalWrite(WHEEL_STBY  , HIGH);

  // Initialize the color sensor
  //colorSensor.setConfiguration(VEML6040_SD_ENABLE);
  //colorSensor.setConfiguration(VEML6040_IT_320MS + VEML6040_AF_AUTO
  //                                 + VEML6040_SD_ENABLE);
  // Talk to testing to get the correct configuration
  Serial3.begin(115200);
  Serial.begin(115200);
  Serial3.println("Starting Up...");
  //Wire.begin();
  /*
  if(!colorSensor.begin()) {
    digitalWrite(LEDY, HIGH);
  }
  */
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

bool atMiddle() {
  bool exclusivityBool = true;
  for(int i = 0; i < 3; ++i) {
    exclusivityBool &= (sensors[i] == LOW && sensors[7 - i] == LOW);
  }
  return (sensors[TARGET_INDEX] == HIGH && sensors[TARGET_INDEX + 1] == HIGH
          && exclusivityBool);
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
  writeToWheels(leftSpeed % 255, rightSpeed % 255);

  // Return true if the sensors can see a fork
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

/*
  Proven to work by cases.
*/
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

/*
  Inductively proven to work.
*/
bool followTrackState() {
  static int state = 0;
  printVar = state;
  bool isFinished = false;

  switch(state) {
    case 3:
      rightArm.write(50);
      break;
    case 9:
    case 10:
      leftArm.write(145);
      break;
    case 14:
      isFinished = amountSeen > TURN_AMOUNT;
      break;
    default:
      rightArm.write(100);
      leftArm.write(100);
      break;
  }

  if(doTurnSequence(TURN_SEQUENCE, state))
    state++;
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
      if(swoopTurn(dir, -255, 2150))  state ++;
      break;
    case 1:
      if(backToCornerState(100, 10)) {
        state++;

        //writeWheelDirection(WHEEL_FORWARDS, WHEEL_FORWARDS);
      }
      break;
    case 2:
      writeToWheels(0, 0);
      // Add the drop off
			if(dir == RIGHT) {
				leftDump.write(DUMP_POS);//added 90 to initial position
			} else {
				rightDump.write(DUMP_POS);
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
      /*
        In this state, we want to swing the robot to the right. It needs to 
        turn to the right so that we can get it to the straight line beside 
        the corner. We exit this state after a couple of milliseconds of delay. 
        Once this state is over, the robot should be pointed towards the line
        and ready to go onward to the line. 
      */
      leftDump.write(DONT_DUMP_POS);

      if(turnFromWall(150)) { //TODO: tweak delay value
        state++;
        // state = 0;
        // ^- if you want to test just this state, use state = 0, not state++
      }
      break;
    case 4: // after turning from the wall, in this state we find the line
      /*
        This state takes the robot to the line and stops when you actually find
        the line. 
      */
      if(findLine(150)) {
        state++;
        // state = 0;
        // ^- if you want to test just this state, use state = 0, not state++ 
      }
      break;
    case 5: // this state gets the robot back into line following
      /*
        In this state, we need to straighten ourselves out a bit before we 
        kick into line following in the next state. So, we are trying to do a 
        turn here to treat it like a fork. I don't know that this will work. 
        So, you might need to add a new state that does a gradual turn to the 
        left for a couple of milliseconds to get it in a suitable line follow 
        state. 
      */
      if(turn(50, LEFT)) {
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

  return state == 3 && amountSeen > TURN_AMOUNT;
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

bool turnFromWall(int d) {
	writeToWheels(150,0);
	return delayState(d);
}

/*
                               COLOR SORTING CODE:

  The following functions all relate to color sorting.

bool isBallPresent() {
  return colorSensor.getWhite() > BALL_TRIGGER;
  // Talk to testing to figure out what signifies where there's a ball

}

int getPositionFromBall() {
  if(isBallPresent()) {
    return (colorSensor.getWhite() > ORANGE_BALL) ? ORANGE_POS : WHITE_POS;
  } else {
    return NO_BALL;
  }
}
*/

/*
  Color sorting code:
    Most of this should work. Just change the sorting in sortBalls so it changes
    the position conditionally.
*/

bool sort(int color) {
  sorter.write(color);
  return delayState(250);
}

void sortBalls(bool turning) {
  static bool sorting = false;
  if(sorting) {
    sorting = !sort(ORANGE); // this is the same
  } else {
    sorter.write(PICK_UP);
    sorting = turning; // sorting = isBallPresent();
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
      for(int i = 50; i < 150; i += 5) {
        sorter.write(i);
        delay(2000);
        printVar = i;
      }
      break;
  }

  iterations++;
  if(iterations == BLUETOOTH_LIMITER)
  {
    iterations = 0;
    Serial3.print("[ ");
    for(int i = 0; i < 8; i++)
    {
      Serial3.print(sensors[i]);
      Serial3.print(" ");
    }

    Serial3.println("]");
    Serial3.print("State: ");
    Serial3.println(state);
    Serial3.print("Substate: ");
    Serial3.println(printVar);
    /*Serial3.println("Backup left: ");
    Serial3.println(readBackLeft());*/
    Serial3.println("----------------------------------------");
    /*Serial3.print("Get Data = ");
    Serial3.print(getBallData());
    Serial3.print(" White val: ");
    Serial3.println(colorSensor.getBlue());*/

    /*
    int b = colorSensor.getBlue();
    int w = colorSensor.getWhite();
    int g = colorSensor.getGreen();
    int r = colorSensor.getRed();
    Serial3.print("Blue: ");
    Serial3.println(b);
    Serial3.print("Red: ");
    Serial3.println(r);
    Serial3.print("Green: ");
    Serial3.println(g);
    Serial3.print("White: ");
    Serial3.println(w);
    */
  }
}

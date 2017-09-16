#include <Arduino.h>
#include "pins.h"
#include "PT6961.h"

int count = 0;
PT6961 display(DIN, CLOCK, CS);

int firstIndex = 0;
int amountSeen = 0;
int state = 0;

int sensors[8] = {0};
bool isRunning = false;
bool turning   = false;

const int TURNLEN               = 14;
char      turnSequence[TURNLEN] = { 'L', 'R', 'L', 'L', 'L', 'R', 'L', 'L', 
                                    'R', 'R', 'R', 'R', 'L', 'L' };
int       turnPointer           = 0;

void setup() {
  // put your setup code here, to run once:
  display.initDisplay();

  pinMode(SENSOR_0, INPUT);
  pinMode(SENSOR_1, INPUT);
  pinMode(SENSOR_2, INPUT);
  pinMode(SENSOR_3, INPUT);
  pinMode(SENSOR_4, INPUT);
  pinMode(SENSOR_5, INPUT);
  pinMode(SENSOR_6, INPUT);
  pinMode(SENSOR_7, INPUT);

  pinMode(GO_BUTTON, INPUT_PULLUP);
  
  // Motor controller pins for the right motor
  pinMode(MC_PWMA, OUTPUT);
  pinMode(MC_AIN2, OUTPUT);
  pinMode(MC_AIN1, OUTPUT);
  
  // Motor controller pins for the left motor
  pinMode(MC_PWMB, OUTPUT);
  pinMode(MC_BIN1, OUTPUT);
  pinMode(MC_BIN2, OUTPUT);

  /*
   * The following output configurations set both motors 
   * to move forward. The two Robot solution doesn't require
   * backwards movement, so the wheels should be permanently
   * forward.
   *
   * - Note: when the new boards are installed, this code will be 
   *        changed because the board is going to tie these to high
   *  - Issues here the right wheel is only spinning backwards
   */
  digitalWrite(MC_AIN1, LOW);
  digitalWrite(MC_AIN2, HIGH);      
  digitalWrite(MC_BIN1, HIGH);
  digitalWrite(MC_BIN2, LOW);
}

void readLine()
{
  sensors[0] = digitalRead(SENSOR_0);
  sensors[1] = digitalRead(SENSOR_1);
  sensors[2] = digitalRead(SENSOR_2);
  sensors[3] = digitalRead(SENSOR_3);
  sensors[4] = digitalRead(SENSOR_4);
  sensors[5] = digitalRead(SENSOR_5);
  sensors[6] = digitalRead(SENSOR_6);
  sensors[7] = digitalRead(SENSOR_7);
}

void extrapolateData() {
  firstIndex = 0;
  amountSeen = 0;
  for(int i = 7; i >= 0; i--){
    if(sensors[i] == 1){
      firstIndex = i;
      amountSeen++;
    }
  } 
}

void writeToWheels(int leftSpeed, int rightSpeed) {
  analogWrite(MC_PWMA, rightSpeed);
  analogWrite(MC_PWMB, leftSpeed); 
}

bool lineFollow(int ts, int strictness){
  int offset = 3 - firstIndex;
  int rightWheel = ts + offset * strictness;
  int leftWheel = ts - offset * strictness;
  writeToWheels(leftWheel, rightWheel);
  return amountSeen >= 5;
}

bool turn(char dir, int ts, int strictness)
{
  if(dir == 'L') {
    writeToWheels(ts - strictness, ts + strictness);
  } else {
    writeToWheels(ts + strictness, ts - strictness);
  }
  return amountSeen <= 3;
}

void loop() {
  // put your main code here, to run repeatedly:
  readLine();
  extrapolateData();
  count++;
  if(count % 131 == 0){
    display.sendDigits((char)firstIndex,  (char)amountSeen, state, turnPointer, 
                       (char)isRunning);
  }
  
  if(isRunning) {
    if(turnPointer == TURNLEN) {
      isRunning = false;
      turnPointer = 0;
      state = 0;
    }

    // lineFollow()
    if(!turning) {
      turning = lineFollow(70, 30);
    } else if(turn(turnSequence[turnPointer], 50, 50)) {
      turnPointer++;
      turning = !turning;
    }

  } else {
    writeToWheels(0, 0);
    if(digitalRead(GO_BUTTON) == LOW) {
      isRunning = true;
    }
  }
}

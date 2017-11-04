#include <Servo.h>
#include "pins.h"

Servo dumpServo;

void setup() {

  // Attach Servos to Pins
  dumpServo.attach(DUMP_SERVO);

  // Initialize Buttons
  pinMode(GO_BUTTON, INPUT_PULLUP);
  pinMode(STOP_BUTTON, INPUT_PULLUP);
}

void loop() {
  if(digitalRead(GO_BUTTON) == LOW){
    dumpServo.write(50);
  }
   if(digitalRead(STOP_BUTTON) == LOW){ 
      dumpServo.write(100);
  }
}


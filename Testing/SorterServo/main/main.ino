#include "pinNumbers.h"
#include "constants.h"
#include <Servo.h>
#include <Arduino.h>

Servo sorter;

int command = 65;

void setup() {
  // put your setup code here, to run once:
  //LEDs on shield
  pinMode(LEDG, OUTPUT);
  pinMode(LEDY, OUTPUT);
  digitalWrite(LEDG, LOW);
  digitalWrite(LEDY, LOW);

  //Button Code
  pinMode(BUTTON1, INPUT);
  pinMode(BUTTON2, INPUT);

  //Talking to the Robot
  Serial.begin(115200);


  sorter.attach(SORTER);

}

void loop() {
  // put your main code here, to run repeatedly:
  
  //Serial.println("waiting....");
  delay(998);
  /*if(Serial.available() > 0){
    command = Serial.read();
  }*/
  if(Serial.available()> 0){
    command = Serial.read();
    Serial.println(command);
  }
  
  if (command == 65) {
    sorter.write(CLOSED);
    digitalWrite(LEDG, LOW);
    digitalWrite(LEDY, HIGH);
  }
  else if (command == 66){
    sorter.write(HALF);
    digitalWrite(LEDG, HIGH);
    digitalWrite(LEDY, HIGH);
  }
  else if(command == 67){
    sorter.write(OPEN);
    digitalWrite(LEDG, HIGH);
    digitalWrite(LEDY, LOW);
  }
}


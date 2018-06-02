#ifndef CONSTANTS

#define CONSTANTS

//sorter constants
const int ORANGE = 135;
const int PICK_UP = 98;
const int WHITE = 65;
const int BALL_TRIGGER = 5000;
const int SORT_TIME = 50;

//speed constants
const int SLOW_SPEED = 50;
const int HALF_SPEED = 100;
const int FULL_SPEED = 150;

//Joel constants
const int TARGET_INDEX = 3;
const int TURN_AMOUNT  = 3;
const char LEFT = 'l';
const char RIGHT = 'r';

// Dasch constants
const int  BLUETOOTH_LIMITER = 4000;
const bool WHEEL_FORWARDS   = true;
const bool WHEEL_BACKWARDS  = false;
const bool FORWARDS         = true;
const bool BACKWARDS        = false;

const int R_DONT_DUMP = 100;
const int L_DONT_DUMP = 32;
const int R_DO_DUMP = 20;
const int L_DO_DUMP = 95;

// arm constants
const int L_ARM_DOWN = 90;
const int R_ARM_DOWN = 159;
const int L_ARM_UP = 180;
const int R_ARM_UP = 75;
const int L_ARM_HALF = 115;
const int R_ARM_HALF = 130;

// Turn sequence
const char TURN_SEQUENCE[] = {
  RIGHT, LEFT, LEFT, RIGHT, LEFT, LEFT, LEFT, LEFT, RIGHT, RIGHT, LEFT, RIGHT, 
  RIGHT, RIGHT, LEFT, LEFT
};

const char SECOND_TURN_SEQUENCE[] = {
  RIGHT, RIGHT
};

const char TEST_SEQUENCE[] = { RIGHT, LEFT, LEFT };


#endif

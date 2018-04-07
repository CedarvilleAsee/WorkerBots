#ifndef CONSTANTS

#define CONSTANTS

//sorter constants
const int ORANGE = 48;
const int PICK_UP = 85;
const int WHITE = 115;
const int BALL_TRIGGER = 5000;
const int SORT_TIME = 750;

//speed constants
const int SLOW_SPEED = 70;
const int HALF_SPEED = 170;
const int FULL_SPEED = 255;

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

const int DONT_DUMP_POS = 52;
const int DUMP_POS      = 142;

// Turn sequence
const char TURN_SEQUENCE[] = {
  LEFT, RIGHT, LEFT, LEFT, LEFT, LEFT, RIGHT, RIGHT, LEFT, RIGHT, RIGHT, RIGHT, LEFT, LEFT
};

const char SECOND_TURN_SEQUENCE[] = {
  RIGHT, RIGHT, RIGHT
};


#endif

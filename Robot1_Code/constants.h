#ifndef CONSTANTS

#define CONSTANTS

//sorter constants
const int ORANGE = 30;
const int PICK_UP = 103;
const int WHITE = 85;

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

const int ORANGE_POS  = 50;
const int WHITE_POS   = 70;
const int BLUE_BALL   = 70;
const int ORANGE_BALL = 2100;
const int NO_BALL     = 125;

const int BALL_TRIGGER = 1200;
const int BLUE_TRIGGER = 3000;

const int DONT_DUMP_POS = 52;
const int DUMP_POS      = 142;

// Turn sequence
const char TURN_SEQUENCE[] = {
  LEFT, RIGHT, LEFT, LEFT, LEFT, LEFT, RIGHT, RIGHT, LEFT, RIGHT, RIGHT, RIGHT, LEFT, LEFT
};

const char SECOND_TURN_SEQUENCE[] = {
  RIGHT, RIGHT, RIGHT, RIGHT
};


#endif

// TODO: these constants are pulled from Robot1 as well
#ifndef CONSTANTS

#define CONSTANTS

//sorter constants
const int ORANGE = 30;
const int PICK_UP = 150;
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
const int BLUETOOTH_LIMITER = 4000;

// Turn sequence
const char TURN_SEQUENCE[] = {
  LEFT, RIGHT, LEFT, LEFT, LEFT, LEFT, RIGHT, RIGHT, LEFT, RIGHT, RIGHT, RIGHT, LEFT, LEFT
};

#endif

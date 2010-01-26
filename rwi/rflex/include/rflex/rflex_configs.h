#ifndef RFLEX_CONFIGS_H
#define RFLEX_CONFIGS_H

#include <math.h>

// Arbitrary units per meter
#define ODO_DISTANCE_CONVERSION 103000
// Arbitrary units per radian
#define ODO_ANGLE_CONVERSION 35000
// Arbitrary units per meter
#define RANGE_CONVERSION 1476

#define DEFAULT_TRANS_ACCELERATION 0.7
#define DEFAULT_ROT_ACCELERATION .017

#define POWER_OFFSET 1.2

#define SONAR_ECHO_DELAY 30000
#define SONAR_PING_DELAY 0
#define SONAR_SET_DELAY  0
#define SONAR_MAX_COUNT 224
#define SONAR_COUNT 48
#define SONAR_AGE 1
#define SONAR_MAX_RANGE 3000

#define SONAR_NUM_BANKS 14
const int SONARS_PER_BANK[] = {4, 4, 4, 4, 4, 4,
                               3, 3, 3, 3, 3, 3, 3, 3
                              };
#define SONAR_MAX_PER_BANK 16
#define SONAR_RING_COUNT 2
const int SONARS_PER_RING[] = {24, 24};
const float SONAR_RING_START_ANGLE[] = {172.5, 240.};
const float SONAR_RING_ANGLE_INC[] = {-15, -15};
const float SONAR_RING_DIAMETER[] = {.25, .26};
const float SONAR_RING_HEIGHT[] = {0.055, -0.06};

#define HEADING_HOME_ADDRESS 0x31
#define BUMPER_ADDRESS 0x40
#define BUMPER_COUNT 14
#define BUMPER_ADDRESS_STYLE 0
#define BUMPER_BIT_STYLE 1
#define BUMPER_STYLE 0

#define IR_POSES_COUNT 24
#define IR_BASE_BANK 0
#define IR_BANK_COUNT 6
#define IR_PER_BANK 4

#define USE_JOYSTICK 0
#define JOY_POS_RATIO 6
#define JOY_ANG_RATIO -0.01

#define BASE_OFFSET 0.15
#define BODY_OFFSET 0.485
#define LASER_OFFSET -0.275
#define PTU_X_OFFSET 0.09
#define PTU_Z_OFFSET .755


#endif










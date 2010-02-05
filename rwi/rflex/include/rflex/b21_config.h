#ifndef B21_CONFIG_H
#define B21_CONFIG_H

// Arbitrary units per meter
#define ODO_DISTANCE_CONVERSION 103000
// Arbitrary units per radian
#define ODO_ANGLE_CONVERSION 35000
// Arbitrary units per meter
#define RANGE_CONVERSION 1476

#define POWER_OFFSET 1.2

#define SONAR_ECHO_DELAY 30000
#define SONAR_PING_DELAY 0
#define SONAR_SET_DELAY  0
#define SONAR_MAX_RANGE 3000

#define BODY_INDEX 0
#define BASE_INDEX 1
const int SONARS_PER_BANK[] = {4, 4, 4, 4, 4, 4,
                               3, 3, 3, 3, 3, 3, 3, 3
                              };
const int SONAR_RING_BANK_BOUND[] = {0, 6, 14};
#define SONAR_MAX_PER_BANK 16

const int SONARS_PER_RING[] = {24, 24};
const float SONAR_RING_START_ANGLE[] = {352.5, 240.};
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

#define HOME_BEARING -967561

#endif










/*  ATRVJR constants
 *  Modified from
 *  David Lu!! - 2/2010
 *  Modified from Player Driver
 *
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000
 *     Brian Gerkey, Kasper Stoy, Richard Vaughan, & Andrew Howard
 *
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef ATRVJR_CONFIG_H
#define ATRVJR_CONFIG_H

// Odometery Constants
// ===================
// Arbitrary units per meter
#define ODO_DISTANCE_CONVERSION 90810
// Arbitrary units per radian
#define ODO_ANGLE_CONVERSION 37000

// Sonar Constants
// ===============
// Arbitrary units per meter (for sonar)
#define RANGE_CONVERSION 1476
#define SONAR_ECHO_DELAY 30000
#define SONAR_PING_DELAY 0
#define SONAR_SET_DELAY  0
#define SONAR_MAX_RANGE 3000

#define BODY_INDEX 0
#define BASE_INDEX 1

#warning Sonar, bumper and IR configurations are not updated for use with ATRVJR
const int SONARS_PER_BANK[] = {5, 8, 5, 8 };
const int SONAR_RING_BANK_BOUND[] = {0, 6, 14};
#define SONAR_MAX_PER_BANK 16

const int SONARS_PER_RING[] = {24, 24};
const float SONAR_RING_START_ANGLE[] = {180,90};
const float SONAR_RING_ANGLE_INC[] = {-15, -15};
const float SONAR_RING_DIAMETER[] = {.25, .26};
const float SONAR_RING_HEIGHT[] = {0.055, -0.06};

// Digital IO constants
// ====================
#define HOME_BEARING -32500

#define BUMPER_COUNT 14
#define BUMPER_ADDRESS_STYLE 0
#define BUMPER_BIT_STYLE 1
#define BUMPER_STYLE 0
const int BUMPERS_PER[] = {6,8};
const double BUMPER_ANGLE_OFFSET[] = {-1,1,-1,1};
const double BUMPER_HEIGHT_OFFSET[][4] = {{.5,.5,.05,.05},
    {.25,.25,.05,.05}
};

// IR Constants
// ============
#define IR_POSES_COUNT 24
#define IR_BASE_BANK 0
#define IR_BANK_COUNT 6
#define IR_PER_BANK 4

// Misc Constants
// ==============
#define USE_JOYSTICK 0
#define JOY_POS_RATIO 6
#define JOY_ANG_RATIO -0.01
#define POWER_OFFSET 1.2
#define PLUGGED_THRESHOLD 25.0

#endif










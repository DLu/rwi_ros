#include <rflex/b21_driver.h>
#include <rflex/b21_config.h>
#include <math.h>

float B21::getDistance() {
    return distance / (float) ODO_DISTANCE_CONVERSION;
}

float B21::getBearing() {
    return (bearing - HOME_BEARING) / (float) ODO_ANGLE_CONVERSION;
}

float B21::getTranslationalVelocity() {
    return t_vel / (float) ODO_DISTANCE_CONVERSION;
}

float B21::getRotationalVelocity() {
    return r_vel / (float) ODO_ANGLE_CONVERSION;
}

float B21::getVoltage() {
    return voltage/100.0 + POWER_OFFSET;
}

int B21::getNumBodySonars() {
    return SONARS_PER_RING[BODY_INDEX];
}

int B21::getNumBaseSonars() {
    return SONARS_PER_RING[BASE_INDEX];
}

void B21::getBodySonarReadings(float* readings) {
    getSonarReadings(BODY_INDEX, readings);
}

void B21::getBaseSonarReadings(float* readings) {
    getSonarReadings(BASE_INDEX, readings);
}

void B21::getBodySonarPoints(sensor_msgs::PointCloud* cloud) {
    getSonarPoints(BODY_INDEX, cloud);
}
void B21::getBaseSonarPoints(sensor_msgs::PointCloud* cloud) {
    getSonarPoints(BASE_INDEX, cloud);
}

void B21::setSonarPower(bool on) {
    unsigned long echo = on?SONAR_ECHO_DELAY:0,
                         ping = on?SONAR_PING_DELAY:0,
                                set  = on?SONAR_SET_DELAY:0,
                                       val  = on?2:0;
    configureSonar(echo, ping, set, val);
}


void B21::setMovement( float tvel, float rvel,
                       float acceleration ) {
    set_velocity(tvel * ODO_DISTANCE_CONVERSION,
                 rvel * ODO_ANGLE_CONVERSION,
                 acceleration * ODO_DISTANCE_CONVERSION);
}


void B21::getSonarReadings(int ringi, float* adjusted_ranges) {
    int i = 0;
    for (int x = SONAR_RING_BANK_BOUND[ringi];x<SONAR_RING_BANK_BOUND[ringi+1];x++) {
        for (int y=0;y<SONARS_PER_BANK[x];y++) {
            int range = sonar_ranges[x*SONAR_MAX_PER_BANK+y];
            if (range > SONAR_MAX_RANGE)
                range = SONAR_MAX_RANGE;
            float fRange = range / (float) RANGE_CONVERSION;
            adjusted_ranges[i] = fRange;
            i++;
        }
    }
}

void B21::getSonarPoints(int ringi, sensor_msgs::PointCloud* cloud) {
    int numSonar = SONARS_PER_RING[ringi];
    float* readings = new float[numSonar];
    getSonarReadings(ringi, readings);
    cloud->set_points_size(numSonar);
    int c = 0;
    for (int i = 0; i < numSonar; ++i) {
        if (readings[i] < SONAR_MAX_RANGE) {
            double angle =  SONAR_RING_START_ANGLE[ringi] + SONAR_RING_ANGLE_INC[ringi]*i;
            angle *= M_PI / 180.0;
            double d = SONAR_RING_DIAMETER[ringi] + readings[i];
            d *= RANGE_CONVERSION;
            cloud->points[c].x = cos(angle)*d;
            cloud->points[c].y = sin(angle)*d;
            cloud->points[c].x = SONAR_RING_HEIGHT[ringi];
            c++;
        }
    }

}


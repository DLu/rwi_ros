#ifndef B21_DRIVER_H
#define B21_DRIVER_H
#include <rflex/rflex_driver.h>
#include <sensor_msgs/PointCloud.h>

class B21 : public RFLEX {
    private:
        void getSonarReadings(int, float*);
        void getSonarPoints(int, sensor_msgs::PointCloud*);
    public:
        void setSonarPower(bool);
        /*		float getDistance() { return distance / (float) ODO_DISTANCE_CONVERSION; }
        		float getBearing() { return (bearing - HOME_BEARING) / (float) ODO_ANGLE_CONVERSION; }
        		float getTranslationalVelocity() { return t_vel / (float) ODO_DISTANCE_CONVERSION; }
        		float getRotationalVelocity() { return r_vel / (float) ODO_ANGLE_CONVERSION; }
        		float getVoltage() { return voltage/100.0 + POWER_OFFSET; }

        		int getNumBodySonars() { return SONARS_PER_RING[BODY_INDEX]; }
        		int getNumBaseSonars() { return SONARS_PER_RING[BASE_INDEX]; }
        		float* getBodySonarReadings() { return getSonarReadings(BODY_INDEX); }
        		float* getBaseSonarReadings() { return getSonarReadings(BASE_INDEX); }
        */
        float getDistance();
        float getBearing();
        float getTranslationalVelocity();
        float getRotationalVelocity();
        float getVoltage();

        int getNumBodySonars();
        int getNumBaseSonars();
        void getBodySonarReadings(float* readings);
        void getBaseSonarReadings(float* readings);
        void getBodySonarPoints(sensor_msgs::PointCloud* cloud);
        void getBaseSonarPoints(sensor_msgs::PointCloud* cloud);
        void setMovement(float t_vel, float r_vel,
                         float acceleration);
};

#endif


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
        float getDistance();
        float getBearing();
        float getTranslationalVelocity();
        float getRotationalVelocity();
        float getVoltage();
        int getNumBodySonars();
        int getNumBaseSonars();
        void getBodySonarReadings(float*);
        void getBaseSonarReadings(float*);
        void getBodySonarPoints(sensor_msgs::PointCloud*);
        void getBaseSonarPoints(sensor_msgs::PointCloud*);
        void setMovement(float, float, float);
};

#endif


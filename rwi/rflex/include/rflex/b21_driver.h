#ifndef B21_DRIVER_H
#define B21_DRIVER_H

/*
 *  B21 Driver - By David Lu!! 2/2010
 *  Modified from Player code
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

#include <rflex/rflex_driver.h>
#include <sensor_msgs/PointCloud.h>

class B21 : public RFLEX {
    public:
        B21();
        virtual ~B21();
        void setSonarPower(bool);
        float getDistance() const;
        float getBearing() const;
        float getTranslationalVelocity() const;
        float getRotationalVelocity() const;
        float getVoltage() const;
        bool isPluggedIn() const;
        int getNumBodySonars() const;
        int getNumBaseSonars() const;
        void getBodySonarReadings(float* readings) const;
        void getBaseSonarReadings(float* readings) const;
        void getBodySonarPoints(sensor_msgs::PointCloud* cloud) const;
        void getBaseSonarPoints(sensor_msgs::PointCloud* cloud) const;
        void setMovement(float, float, float);
    private:
        void getSonarReadings(const int ringi, float* readings) const;
        void getSonarPoints(const int ringi, sensor_msgs::PointCloud* cloud) const;

        // Not allowed to use these
        B21(const B21 &b21);
        B21 &operator=(const B21 &b21);
};

#endif


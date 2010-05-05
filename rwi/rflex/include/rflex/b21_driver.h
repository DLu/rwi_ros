#ifndef B21_DRIVER_H
#define B21_DRIVER_H

#include <rflex/rflex_driver.h>
#include <sensor_msgs/PointCloud.h>

/**
 * \brief B21 Driver class
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
class B21 : public RFLEX {
    public:
        B21();
        virtual ~B21();
        void setSonarPower(bool);
        float getDistance();
        float getBearing();
        float getTranslationalVelocity() const;
        float getRotationalVelocity() const;
        float getVoltage() const;
        bool isPluggedIn() const;
        int getNumBodySonars() const;
        int getNumBaseSonars() const;

        /** Get readings from the sonar on the body of the B21
         * in meters
         * \param readings Data structure into which the sonar readings are saved */
        void getBodySonarReadings(float* readings) const;
        /** Get readings from the sonar on the base of the B21
         * in meters
         * \param readings Data structure into which the sonar readings are saved */
        void getBaseSonarReadings(float* readings) const;

        /** Gets a point cloud for sonar readings from body
         * \param cloud Data structure into which the sonar readings are saved */
        void getBodySonarPoints(sensor_msgs::PointCloud* cloud) const;

        /** Gets a point cloud for sonar readings from base
         * \param cloud Data structure into which the sonar readings are saved */
        void getBaseSonarPoints(sensor_msgs::PointCloud* cloud) const;

        /** Gets a point cloud for the bump sensors on the body
         * \param cloud Data structure into which the bump readings are saved
         * \return number of active bump sensors
         */
        int getBodyBumps(sensor_msgs::PointCloud* cloud) const;

        /** Gets a point cloud for the bump sensors on the base
         * \param cloud Data structure into which the bump readings are saved
         * \return number of active bump sensors */
        int getBaseBumps(sensor_msgs::PointCloud* cloud) const;

        /** Sets the motion of the robot
         * \param tvel Translational velocity (in m/s)
         * \param rvel Rotational velocity (in radian/s)
         * \param acceleration Translational acceleration (in m/s/s) */
        void setMovement(float tvel, float rvel, float acceleration);

        /** Processes the DIO packets - called from RFflex Driver
         * \param address origin
         * \param data values */
        void processDioEvent(unsigned char address, unsigned short data);

        /** Detects whether the robot has all the necessary components
         * to calculate odometry
         * \return bool true if robot has read its distance, bearing and home bearing */
        bool isOdomReady() const {
            return odomReady==3;
        }

    private:
        /**\param ringi BODY_INDEX or BASE_INDEX
         * \param readings Data structure into which the sonar readings are saved */
        void getSonarReadings(const int ringi, float* readings) const;
        /**\param ringi BODY_INDEX or BASE_INDEX
         * \param cloud Data structure into which the sonar readings are saved */
        void getSonarPoints(const int ringi, sensor_msgs::PointCloud* cloud) const;

        /**\param index BODY_INDEX or BASE_INDEX
           \param cloud Data structure into which the bump sensors are saved
           \return number of active bump sensors
        */
        int getBumps(const int index, sensor_msgs::PointCloud* cloud) const;

        int first_distance;
        bool found_distance;
        int home_bearing; ///< Last home bearing (arbitrary units)
        int** bumps;

        // Not allowed to use these
        B21(const B21 &b21); 				///< Private constructor - Don't use
        B21 &operator=(const B21 &b21); 	///< Private constructor - Don't use
};

#endif


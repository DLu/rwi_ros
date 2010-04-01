#ifndef RFLEX_DRIVER_H
#define RFLEX_DRIVER_H

#include <rflex/rflex_info.h>
#include <pthread.h>

/**
 * \brief RFLEX Driver to handle input and output to RFlex devices.
 *
 *  RFLEX Driver - 2/2010
 *  Modified from Player code by David Lu!!
 *  Original Input Output code by Bill Smart
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
class RFLEX {
    public:
        RFLEX();
        virtual ~RFLEX();

        /** Opens connection to serial port with specified device name
            \param devname Device name assigned to serial port
            \return -1 on error */
        int initialize(const char* devname);

        /** Configure the sonar parameters and send message to RFlex.
          * \param echoDelay Echo Delay
          * \param pingDelay Ping Delay
          * \param setDelay Set Delay
          * \param val Unknown
          * @todo Figure out unknown value's purpose.
          */
        void configureSonar(const unsigned long echoDelay, const unsigned long pingDelay,
                            const unsigned long setDelay, const unsigned long val);

        /** Turn IR on or off
          * \param power true for on, false for off */
        void setIrPower(const bool power);
        /** Turn Brake on or off
          * Note: Brake on means the controller cannot move the robot
          *    and external forces CAN move it.
          * \param power true for on, false for off */
        void setBrakePower(const bool power);

        /** Set the frequency that the Digital IO devices are checked.
          * \param period Period in milliseconds
          */
        void setDigitalIoPeriod(const long period);

        /** Set the frequency that the odometry is checked.
          * \param period Period in milliseconds
          */
        void setOdometryPeriod(const long period);

        /** Sends a set motion defaults message to the device. */
        void motionSetDefaults();

        /** Gets brake power
           * \return True if brake is engaged */
        bool getBrakePower() const {
            return brake;
        }

        /** Gets the number of IR sensors
         * \return Number of IR sensors
         */
        int  getIrCount() const {
            return numIr;
        }

        /** Sets the velocity
         * \param transVelocity Translational velocity in arbitrary units
         * \param rotVelocity Rotational velocity in arbitrary units
         * \param acceleration Acceleration (also in arbitrary units) */
        void setVelocity(const long transVelocity, const long rotVelocity,
                         const long acceleration);

        /** Sends a system status command to the device.
         * Updates the brake and battery status. */
        void sendSystemStatusCommand();

    protected:

        virtual void processDioEvent(unsigned char address, unsigned short data);

        int distance;			///< Raw translational odometry
        int bearing;			///< Raw rotational odometry
        int transVelocity;		///< Raw translational velocity
        int rotVelocity;		///< Raw rotational velocity

        int sonar_ranges[SONAR_MAX_COUNT];	///< Raw Sonar readings (including unconnected ports)
        long voltage;	///< Raw voltage reading
        bool brake;		///< Brake Status

        unsigned short dioData[24];	///< Storage for digital IO values

        int lcdX, lcdY;
        unsigned char * lcdData;

        int numIr; ///< Number of IR sensors
        unsigned char * irRanges; ///< Raw values from IR sensors
        int home_bearing_found;
        int odomReady;

    private:
        void parsePacket(const unsigned char* buffer);
        void parseMotReport(const unsigned char* buffer);
        void parseDioReport(const unsigned char* buffer);
        void parseIrReport(const unsigned char* buffer);
        void parseSysReport(const unsigned char* buffer);
        void parseSonarReport(const unsigned char* buffer);
        void parseJoyReport(const unsigned char* buffer);

        // IO Stuff
        int fd;				///< File descriptor for serial port
        pthread_t thread;	///< Thread which reads input upon arrival
        pthread_mutex_t writeMutex; ///< Mutex around writing to port

        unsigned char readBuffer[BUFFER_SIZE];
        unsigned char writeBuffer[BUFFER_SIZE];

        bool found;
        int offset;

        static void *readThread(void *ptr); ///< Read Thread

        /**
          * Send a command to the serial port
         * \param port Should be one of these: SYS_PORT, MOT_PORT, JSTK_PORT, SONAR_PORT, DIO_PORT, IR_PORT
         * \param id
         * \param opcode See opcodes in rflex_info.h
         * \param length length of the data
         * \param data actual data */
        bool sendCommand(const unsigned char port, const unsigned char id, const unsigned char opcode, const int length, unsigned char* data);

        void readPacket(); ///< After reading the data, it checks for errors and then parses
        int readData();    ///< Reads in a packet until it finds and end of packet signal
        bool writePacket(const int length) const; ///< Writes packet currently in write buffer to device
        unsigned char computeCRC(const unsigned char *buffer, const int n); ///< Calculates error checking code for specified buffer

        // Not allowed to use these
        RFLEX(const RFLEX &rflex); 				///< Private constructor - Don't use
        RFLEX &operator=(const RFLEX &rflex);	///< Private constructor - Don't use
};
#endif

/*
 *  RFLEX Driver - By David Lu!! 2/2010
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

#ifndef RFLEX_DRIVER_H
#define RFLEX_DRIVER_H

#include <rflex/serial.h>

class RFLEX {
    protected:
        int openConnection(const char *);

        int  parsePacket(RFlexPacket*);
        void parseMotReport(RFlexPacket*);
        void parseDioReport(RFlexPacket*);
        void parseIrReport(RFlexPacket*);
        void parseSysReport(RFlexPacket*);
        void parseSonarReport(RFlexPacket*);
        void parseJoyReport(RFlexPacket*);

        int distance, bearing, transVelocity, rotVelocity;
        int *sonar_ranges;
        int**sonar_history;
        long voltage;
        bool brake;

        unsigned short dioData[24];
        int lcdX, lcdY;
        unsigned char * lcdData;

        int numIr;
        unsigned char * irRanges;
        int home_bearing_found;

        SerialPort* serial;

    public:
        int initialize(const char* devname);
        int closeConnection();

        void configureSonar(unsigned long echoDelay, unsigned long pingDelay,
                            unsigned long setDelay, unsigned val);
        void setIrPower(bool);
        void setBrakePower(bool);
        void setDigitalIoPeriod(long period);
        void setOdometryPeriod(long period);
        void motionSetDefaults();

        bool getBrakePower() {
            return brake;
        }
        int  getIrCount() {
            return numIr;
        }

        void setVelocity(long transVelocity, long rotVelocity,
                         long acceleration);
        void sendSystemStatusCommand();
        void parsePackets();

};
#endif

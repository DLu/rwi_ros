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
#include <rflex/rflex_info.h>

class RFLEX {
    public:
        RFLEX();
        virtual ~RFLEX();

        int initialize(const char* devname);

        void configureSonar(const unsigned long echoDelay, const unsigned long pingDelay,
                            const unsigned long setDelay, const unsigned long val);
        void setIrPower(const bool power);
        void setBrakePower(const bool power);
        void setDigitalIoPeriod(const long period);
        void setOdometryPeriod(const long period);
        void motionSetDefaults();

        bool getBrakePower() const {
            return brake;
        }
        int  getIrCount() const {
            return numIr;
        }

        void setVelocity(const long transVelocity, const long rotVelocity,
                         const long acceleration);
        void sendSystemStatusCommand();
        void parsePackets();

    protected:
        int  parsePacket(RFlexPacket* pkt);
        void parseMotReport(RFlexPacket* pkt);
        void parseDioReport(RFlexPacket* pkt);
        void parseIrReport(RFlexPacket* pkt);
        void parseSysReport(RFlexPacket* pkt);
        void parseSonarReport(RFlexPacket* pkt);
        void parseJoyReport(RFlexPacket* pkt);

        int distance, bearing, transVelocity, rotVelocity;
        int sonar_ranges[SONAR_MAX_COUNT];
        long voltage;
        bool brake;

        unsigned short dioData[24];
        int lcdX, lcdY;
        unsigned char * lcdData;

        int numIr;
        unsigned char * irRanges;
        int home_bearing_found;

        SerialPort serial;

    private:
        // Not allowed to use these
        RFLEX(const RFLEX &rflex);
        RFLEX &operator=(const RFLEX &rflex);
};
#endif

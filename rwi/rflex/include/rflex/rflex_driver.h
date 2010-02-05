/*  Player - One Hell of a Robot Server
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
        int open_connection(const char *);

        int  parsePacket(RFlexPacket*);
        void parseMotReport(RFlexPacket*);
        void parseDioReport(RFlexPacket*);
        void parseIrReport(RFlexPacket*);
        void parseSysReport(RFlexPacket*);
        void parseSonarReport(RFlexPacket*);
        void parseJoyReport(RFlexPacket*);

        int distance, bearing, t_vel, r_vel;
        int *sonar_ranges;
        int**sonar_history;

        long voltage;
        bool brake;

        unsigned short dio_data[24];

        int lcd_x, lcd_y;
        unsigned char * lcd_data;

        int num_ir;
        unsigned char * ir_ranges;

        int home_bearing_found;

        SerialPort* serial;

    public:
        int initialize(const char* devname);
        int close_connection();

        void configureSonar(unsigned long echo_delay, unsigned long ping_delay,
                            unsigned long set_delay, unsigned val);
        void setIrPower(bool);
        void setBrakePower(bool);
        void setDigitalIoPeriod(long period);
        void setOdometryPeriod(long period);
        void motion_set_defaults();

        int getRawDistance() {
            return distance;
        }
        int getRawBearing() {
            return bearing;
        }
        int getRawTranslationalVelocity() {
            return t_vel;
        }
        int getRawRotationalVelocity() {
            return r_vel;
        }
        long getRawVoltage() {
            return voltage;
        }
        bool getBrakePower() {
            return brake;
        }
        int* getRawSonar() {
            return sonar_ranges;
        }
        int  getIrCount() {
            return num_ir;
        }
        unsigned char* getRawIr() {
            return ir_ranges;
        }

        void set_velocity(long t_vel, long r_vel,
                          long acceleration);
        void sendSystemStatusCommand();
        void parsePackets();

};
#endif

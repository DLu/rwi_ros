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

#ifndef RFLEX_COMMANDS_H
#define RFLEX_COMMANDS_H

#include <rflex/serial.h>

class RFLEX {
    private:
        int open_connection(const char *);

        int  parsePacket(RFlexPacket*);
        void parseMotReport(RFlexPacket*);
        void parseDioReport(RFlexPacket*);
        void parseIrReport(RFlexPacket*);
        void parseSysReport(RFlexPacket*);
        void parseSonarReport(RFlexPacket*);
        void parseJoyReport(RFlexPacket*);

        int distance, bearing, t_vel, r_vel;
        int *ranges, *oldranges;

        int num_bumpers;
        char *bumpers;

        long voltage;
        bool brake;

        int lcd_x, lcd_y;
        unsigned char * lcd_data;

        int num_ir;
        unsigned char * ir_ranges;

        int home_bearing;
        int home_bearing_found;
        pthread_t m_read_thread;

        SerialPort* serial;

    public:
        int initialize(const char* devname);
        int close_connection();

        void setSonarPower(bool);
        void setIrPower(bool);
        void setBrakePower(bool);
        void setDigitalIoPeriod(long period);
        void setOdometryPeriod(long period);
        void motion_set_defaults();


        void update_status(float *distance,
                           float *bearing, float *t_vel,
                           float *r_vel);

        void update_system(int *battery,
                           int *brake);

        int update_sonar(float **rings);
        void update_bumpers(int num_bumpers,
                            char *values);
        void update_ir(int num_irs,
                       unsigned char *ranges);
        void set_velocityF( float tvelf, float rvelf,
                            float accelerationf );
        void set_velocity(long t_vel, long r_vel,
                          long acceleration);
        void stop_robot(int deceleration);

        void parsePackets();

};
#endif

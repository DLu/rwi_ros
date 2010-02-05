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

#include <rflex/rflex_info.h>
#include <rflex/rflex_driver.h>
#include <rflex/rflex_packet.h>

#include <stdio.h>
#include <stdlib.h>

//finds the sign of a value
static long sgn( long val ) {
    return (val<0)?0:1;
}

static unsigned int getInt16( unsigned char *bytes ) {
    unsigned int i;
    memcpy( &i, bytes, 2 );
    return(htons(i));
}


static unsigned long getInt32( unsigned char *bytes ) {
    unsigned long i;
    memcpy( &i, bytes, 4 );
    return(htonl(i));
}


static void putInt8( unsigned int i, unsigned char *bytes ) {
    memcpy( bytes, &i, 1 );
}

static void putInt32( unsigned long l, unsigned char *bytes ) {
    uint32_t conv;
    conv = htonl( l );
    memcpy( bytes, &conv, 4 );
}

void RFLEX::configureSonar(unsigned long echo_delay, unsigned long ping_delay,
                           unsigned long set_delay, unsigned val) {

    unsigned char data[MAX_COMMAND_LENGTH];
    putInt32( echo_delay, &(data[0]) );
    putInt32( ping_delay, &(data[4]) );
    putInt32( set_delay , &(data[8]) );
    putInt8(  val, &(data[12]) );
    RFlexPacket packet(SONAR_PORT, 4, SONAR_RUN, 13, data );
    serial->sendPacket(&packet);
}

void RFLEX::setIrPower( bool on ) {
    unsigned char data[MAX_COMMAND_LENGTH];
    long v1 = 0,
              v2 = on?70:0,
                   v3 = on?10:0,
                        v4 = on?20:0,
                             v5 = on?150:0;
    unsigned char v6 = on?2:0;

    putInt32( v1, &(data[0]) );
    putInt32( v2, &(data[4]) );
    putInt32( v3, &(data[8]) );
    putInt32( v4, &(data[12]) );
    putInt32( v5, &(data[16]) );
    putInt8(  v6, &(data[20]) );
    RFlexPacket packet(IR_PORT, 0, IR_RUN, 21, data );
    serial->sendPacket(&packet);
}

void RFLEX::setBrakePower( bool on ) {
    RFlexPacket packet(MOT_PORT, 0, on?MOT_BRAKE_SET:MOT_BRAKE_RELEASE, 0, NULL );
    serial->sendPacket(&packet);
}

void RFLEX::motion_set_defaults(  ) {
    RFlexPacket packet(MOT_PORT, 0, MOT_SET_DEFAULTS, 0, NULL );
    serial->sendPacket(&packet);
}

void RFLEX::setDigitalIoPeriod( long period ) {
    unsigned char data[MAX_COMMAND_LENGTH];
    putInt32( period, &(data[0]) );
    RFlexPacket packet(DIO_PORT, 0, DIO_REPORTS_REQ, 4, data );
    serial->sendPacket(&packet);
}

void RFLEX::setOdometryPeriod( long period ) {
    unsigned char data[MAX_COMMAND_LENGTH];
    long mask = period==0?0:3;
    putInt32( period, &(data[0]) );         /* period in ms */
    putInt32( mask, &(data[4]) );           /* mask */
    RFlexPacket packet(MOT_PORT, 0, MOT_SYSTEM_REPORT_REQ, 8, data );
    serial->sendPacket(&packet);
}

void RFLEX::set_velocity( long tvel, long rvel, long acceleration) {
    long utvel =labs(tvel);
    long urvel =labs(rvel);
    unsigned char data[MAX_COMMAND_LENGTH];

    // ** workaround for stupid hardware bug, cause unknown, but this works
    // ** with minimal detriment to control
    // ** avoids all values with 1b in highest or 3'rd highest order byte

    // 0x1b is part of the packet terminating string
    // which is most likely what causes the bug
    /*
        // ** if 2'nd order byte is 1b, round to nearest 1c, or 1a
        if ((urvel&0xff00)==0x1b00) {
            // ** if lowest order byte is>127 round up, otherwise round down
            urvel=(urvel&0xffff0000)|((urvel&0xff)>127?0x1c00:0x1aff);
        }

        // ** if highest order byte is 1b, round to 1c, otherwise round to 1a
        if ((urvel&0xff000000)==0x1b000000) {
            // ** if 3'rd order byte is>127 round to 1c, otherwise round to 1a
            urvel=(urvel&0x00ffffff)|(((urvel&0xff0000)>>16)>127?0x1c000000:0x1aff0000);
        }*/

    putInt8( 0,                 &(data[0]) );       /* forward motion */
    putInt32( utvel,        &(data[1]) );       /* abs trans velocity*/
    putInt32( acceleration,    &(data[5]) );       /* trans acc */
    putInt32( STD_TRANS_TORQUE, &(data[9]) );       /* trans torque */
    putInt8( sgn(tvel),         &(data[13]) );      /* trans direction */

    RFlexPacket tpacket(MOT_PORT, 0, MOT_AXIS_SET_DIR, 14, data );
    serial->sendPacket(&tpacket);

    putInt8( 1,                 &(data[0]) );       /* rotational motion */
    putInt32( urvel,        &(data[1]) );       /* abs rot velocity  */
    /* 0.275 rad/sec * 10000 */
    putInt32( STD_ROT_ACC,      &(data[5]) );       /* rot acc */
    putInt32( STD_ROT_TORQUE,   &(data[9]) );       /* rot torque */
    putInt8( sgn(rvel),         &(data[13]) );      /* rot direction */

    RFlexPacket rpacket(MOT_PORT, 0, MOT_AXIS_SET_DIR, 14, data );
    serial->sendPacket(&rpacket);
}

void RFLEX::sendSystemStatusCommand() {
    RFlexPacket packet(SYS_PORT, 0, SYS_STATUS, 0, NULL );
    serial->sendPacket(&packet);
}

void RFLEX::parseMotReport( RFlexPacket* pkt ) {
    int rv, timeStamp, acc, trq;
    unsigned char axis;
    unsigned char* buffer = pkt->data();

    switch (pkt->getOpcode()) {
    case MOT_SYSTEM_REPORT:
        rv        = getInt32(&(buffer[6]));
        timeStamp = getInt32(&(buffer[10]));
        axis      = buffer[14];
        if (axis == 0) {
            distance = getInt32(&(buffer[15]));
            t_vel = getInt32(&(buffer[19]));
        } else if (axis == 1) {
            bearing = getInt32(&(buffer[15]));
            r_vel = getInt32(&(buffer[19]));
        }
        acc       = getInt32(&(buffer[23]));
        trq       = getInt32(&(buffer[27]));
        break;
    default:
        break;
    }
}

//processes a dio packet from the rflex - dio report includes bump sensors...
void RFLEX::parseDioReport( RFlexPacket* pkt ) {
    unsigned long timeStamp;
    unsigned char length, address;
    unsigned short data;
    unsigned char* buffer = pkt->data();
    length = buffer[5];

    switch (pkt->getOpcode()) {
    case DIO_REPORT:
        if (length < 6) {
            fprintf(stderr, "DIO Data Packet too small\n");
            break;
        }
        timeStamp = getInt32(&(buffer[6]));
        address = buffer[10];
        data = getInt16(&(buffer[11]));

        printf("DIO: address 0x%02x (%d) value 0x%02x (%d)\n", address, address, data, data);
        break;
    default:
        break;
    }
}

// Processes the IR sensor report
void RFLEX::parseIrReport( RFlexPacket* pkt ) {
    /*
    unsigned char* buffer = pkt->data();
    unsigned char length = buffer[5];

    // allocate ir storage if we havent already
    // must be a better place to do this but it works
    if (num_ir== 0 && IR_POSES_COUNT > 0) {
        ir_ranges = new unsigned char[IR_POSES_COUNT];
        if (ir_ranges != NULL)
            num_ir = IR_POSES_COUNT;
        else
            fprintf(stderr,"Error allocating ir range storage in rflex status\n");
    }

    switch (pkt->getOpcode()) {
    case IR_REPORT: {
        if (length < 1) {
            fprintf(stderr, "IR Data Packet too small\n");
            break;
        }

        // get number of IR readings to make
        unsigned char pckt_ir_count = buffer[6];
        if (pckt_ir_count < IR_BASE_BANK)
            pckt_ir_count = 0;
        else
            pckt_ir_count -= IR_BASE_BANK;

        if (pckt_ir_count > IR_BANK_COUNT)
            pckt_ir_count = IR_BANK_COUNT;


        // now actually read the ir data
        int ir_data_index = 0;
        for (int i=0; i < IR_BANK_COUNT && ir_data_index < num_ir; ++i) {
            for (int j = 0; j < IR_PER_BANK && ir_data_index < num_ir; ++j,++ir_data_index) {
                // work out the actual offset in teh packet
                //(current bank + bank offfset) * bank data block size + current ir sensor + constant offset
                int data_index = (IR_BASE_BANK + i) * 13 + j + 11;
                ir_ranges[ir_data_index] = buffer[data_index];
            }
        }

        break;
    }
    default:
        break;
    }*/
#warning IR Unreported
}


//processes a sys packet from the rflex - and saves the data in the
//struct for later use, sys is primarily used for bat voltage & brake status
void RFLEX::parseSysReport( RFlexPacket* pkt ) {
    unsigned char* buffer = pkt->data();
    unsigned long timeStamp;
    unsigned char length = buffer[5];


    switch (pkt->getOpcode()) {
    case SYS_LCD_DUMP:
        // currently designed for 320x240 screen on b21r
        // stored in packed format
        if (length < 6) {
            fprintf(stderr, "Got bad Sys packet (lcd)\n");
            break;
        }

        unsigned char lcd_length, row;
        timeStamp=getInt32(&(buffer[6]));
        row = buffer[10];
        lcd_length = buffer[11];
        if (row > lcd_y || lcd_length > lcd_x) {
            fprintf(stderr,"LCD Data Overflow\n");
            break;
        }

        memcpy(&lcd_data[row*lcd_x],&buffer[12],lcd_length);

        // if we got whole lcd dump to file
        /*			if (row == 239)
        			{
        				FILE * fout;
        				if ((fout = fopen("test.raw","w"))!=0)
        				{
        					for (int y=0; y<lcd_y; ++y)
        					{
        						for (int x=0; x<lcd_x;++x)
        						{
        							unsigned char Temp = lcd_data[y*lcd_x + x];
        							for (int i = 0; i < 8; ++i)
        							{
        								if ((Temp >> i) & 0x01)

        									fprintf(fout,"%c",0x0);
        								else
        									fprintf(fout,"%c",0xFF);
        							}
        						}
        						fprintf(fout,"\n");
        					}
        				}
        				fclose(fout);
        			}*/


        break;

    case SYS_STATUS:
        if (length < 9) {
            fprintf(stderr, "Got bad Sys packet (status)\n");
            break;
        }
        timeStamp=getInt32(&(buffer[6]));
        // raw voltage measurement...needs calibration offset added
        voltage=getInt32(&(buffer[10]));
        brake=buffer[14];

        break;

    default:
        fprintf(stderr,"Unknown sys opcode recieved\n");
    }
}

//processes a sonar packet from the rflex
void RFLEX::parseSonarReport( RFlexPacket* pkt ) {
    unsigned char* buffer = pkt->data();
    int x,smallest;
    int retval, timeStamp, count;
    unsigned char dlen = buffer[5];

    switch (pkt->getOpcode()) {
    case SONAR_REPORT:
        retval    = getInt32(&(buffer[6]));
        timeStamp = getInt32(&(buffer[10]));
        count = 0;
        while ((8+count*3<dlen) && (count<256) && (count < SONAR_MAX_COUNT)) {
            unsigned int sid = buffer[14+count*3];
            int value = getInt16( &(buffer[14+count*3+1]) );

            //shift buffer
            for (x=0;x<SONAR_AGE-1;x++)
                sonar_history[x+1][sid] = sonar_history[x][sid];

            //add value to buffer
            sonar_history[0][sid] = value;

            //find the smallest
            smallest = value;
            for (x=1;x<SONAR_AGE;x++)
                if (smallest>sonar_history[x][sid] && sonar_history[x]>=0)
                    smallest=sonar_history[x][sid];
            //set the smallest in last sonar_age as our value
            sonar_ranges[sid] = smallest;
            count++;
        }
        break;
    default:
        break;
    }
}

//processes a joystick packet from the rflex, and sets as command if
// joystick command enabled
void RFLEX::parseJoyReport( RFlexPacket* pkt ) {
    /*unsigned char* buffer = pkt->data();

    static bool JoystickWasOn = false;

    int x,y;
    unsigned char buttons, dlen = buffer[5];

    switch (pkt->getOpcode()) {
    case JSTK_GET_STATE:
        if (dlen < 13) {
            fprintf(stderr,"Joystick Packet too small\n");
            break;
        }
        x = getInt32(&buffer[10]);
        y = getInt32(&buffer[14]);
        buttons = buffer[18];

        if ((buttons & 1) == 1) {
            JoystickWasOn = true;
            set_velocity((long) (y * JOY_POS_RATIO * ODO_DISTANCE_CONVERSION),
                         (long) (x * JOY_ANG_RATIO * ODO_ANGLE_CONVERSION),
                         (long) (DEFAULT_TRANS_ACCELERATION * ODO_DISTANCE_CONVERSION));
            //RFLEX::joy_control = 5;
        } else if (JoystickWasOn) {
            JoystickWasOn = false;
            set_velocity(0,0,(long) (DEFAULT_TRANS_ACCELERATION * ODO_DISTANCE_CONVERSION));
            //RFLEX::joy_control = 5;
        }

        break;
    default:
        break;
    }*/
#warning joystick control not implemented
}


//parses a packet from the rflex, and decides what to do with it
int RFLEX::parsePacket( RFlexPacket* pkt ) {
    if (!pkt->isValid())
        return 0;

    switch (pkt->getPort()) {
    case SYS_PORT:
        parseSysReport( pkt );
        break;
    case MOT_PORT:
        parseMotReport( pkt );
        break;
    case JSTK_PORT:
        parseJoyReport( pkt );
        break;
    case SONAR_PORT:
        parseSonarReport( pkt );
        break;
    case DIO_PORT:
        parseDioReport( pkt );
        break;
    case IR_PORT:
        parseIrReport( pkt);
        break;
    default:
        break;
    }

    return(1);
}

int RFLEX::open_connection(const char *device_name) {
    serial = new SerialPort();
    return serial->open_connection(device_name, 115200);
}

int RFLEX::close_connection() {
    delete serial;
    return 0;
}

int RFLEX::initialize(const char* devname) {
    int ret0 = open_connection(devname);
    if (ret0<0) return ret0;

    sonar_ranges = (int*) malloc(SONAR_MAX_COUNT*sizeof(int));
    sonar_history = (int**) malloc(SONAR_AGE*sizeof(int*));
    for (int j=0;j<SONAR_AGE;j++) {
        sonar_history[j] = (int*) malloc(SONAR_MAX_COUNT*sizeof(int));
        for (int i=0;i<SONAR_MAX_COUNT;i++)
            sonar_history[j][i] = -1;
    }

    // initialise the LCD dump array
    lcd_data=new unsigned char[320*240/8];
    if (lcd_data != NULL) {
        lcd_x=320/8;
        lcd_y=240;
    }

    // allocate dio
#warning allocate dio
    return 0;
}


void RFLEX::parsePackets() {
    while (serial->hasPackets()) {
        RFlexPacket* pkt = serial->getPacket();
        parsePacket(pkt);
        delete pkt;
    }
}











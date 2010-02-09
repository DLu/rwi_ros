/*
 *  RFlex Driver
 *  David Lu!! - 2/2010
 *  Modified from Player driver
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
#include <rflex/rflex_packet.h>

#include <stdio.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <string.h> // memcpy

//finds the sign of a value
static long sgn( long val ) {
    if (val < 0)
        return 0;
    else
        return 1;
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

RFLEX::RFLEX() {
    // initialise the LCD dump array
    lcdData=new unsigned char[320*240/8];
    if (lcdData != NULL) {
        lcdX=320/8;
        lcdY=240;
    }

    // allocate dio
#warning allocate dio
}

int RFLEX::initialize(const char* device_name) {
    return serial.openConnection(device_name, 115200);
}

RFLEX::~RFLEX() {
    // empty destructor
}

void RFLEX::configureSonar(const unsigned long echo_delay, const unsigned long ping_delay,
                           const unsigned long set_delay, const unsigned long val) {

    unsigned char data[MAX_COMMAND_LENGTH];
    putInt32( echo_delay, &(data[0]) );
    putInt32( ping_delay, &(data[4]) );
    putInt32( set_delay , &(data[8]) );
    putInt8(  val, &(data[12]) );
    RFlexPacket packet(SONAR_PORT, 4, SONAR_RUN, 13, data );
    serial.sendPacket(&packet);
}

void RFLEX::setIrPower( const bool on ) {
    unsigned char data[MAX_COMMAND_LENGTH];
    long v1, v2, v3, v4, v5;
    unsigned char v6;
    if (on) {
        v1 = 0;
        v2 = 70;
        v3 = 10;
        v4 = 20;
        v5 = 150;
        v6 = 2;
    } else {
        v1 = v2 = v3 = v4 = v5 = 0;
        v6 = 0;
    }

    putInt32( v1, &(data[0]) );
    putInt32( v2, &(data[4]) );
    putInt32( v3, &(data[8]) );
    putInt32( v4, &(data[12]) );
    putInt32( v5, &(data[16]) );
    putInt8(  v6, &(data[20]) );
    RFlexPacket packet(IR_PORT, 0, IR_RUN, 21, data );
    serial.sendPacket(&packet);
}

void RFLEX::setBrakePower( const bool on ) {
    int brake;
    if (on)
        brake = MOT_BRAKE_SET;
    else
        brake = MOT_BRAKE_RELEASE;

    RFlexPacket packet(MOT_PORT, 0, brake, 0, NULL );
    serial.sendPacket(&packet);
}

void RFLEX::motionSetDefaults(  ) {
    RFlexPacket packet(MOT_PORT, 0, MOT_SET_DEFAULTS, 0, NULL );
    serial.sendPacket(&packet);
}

void RFLEX::setDigitalIoPeriod( const long period ) {
    unsigned char data[MAX_COMMAND_LENGTH];
    putInt32( period, &(data[0]) );
    RFlexPacket packet(DIO_PORT, 0, DIO_REPORTS_REQ, 4, data );
    serial.sendPacket(&packet);
}

void RFLEX::setOdometryPeriod( const long period ) {
    unsigned char data[MAX_COMMAND_LENGTH];
    long mask;
    if (period==0)
        mask = 0;
    else
        mask = 3;

    putInt32( period, &(data[0]) );         /* period in ms */
    putInt32( mask, &(data[4]) );           /* mask */
    RFlexPacket packet(MOT_PORT, 0, MOT_SYSTEM_REPORT_REQ, 8, data );
    serial.sendPacket(&packet);
}

void RFLEX::setVelocity( const long tvel, const long rvel, const long acceleration) {
    long utvel =labs(tvel);
    long urvel =labs(rvel);
    unsigned char data[MAX_COMMAND_LENGTH];

    putInt8( 0,                 &(data[0]) );       /* forward motion */
    putInt32( utvel,            &(data[1]) );       /* abs trans velocity*/
    putInt32( acceleration,     &(data[5]) );       /* trans acc */
    putInt32( STD_TRANS_TORQUE, &(data[9]) );       /* trans torque */
    putInt8( sgn(tvel),         &(data[13]) );      /* trans direction */

    RFlexPacket tpacket(MOT_PORT, 0, MOT_AXIS_SET_DIR, 14, data );
    serial.sendPacket(&tpacket);

    putInt8( 1,                 &(data[0]) );       /* rotational motion */
    putInt32( urvel,            &(data[1]) );       /* abs rot velocity  */
    /* 0.275 rad/sec * 10000 */
    putInt32( STD_ROT_ACC,      &(data[5]) );       /* rot acc */
    putInt32( STD_ROT_TORQUE,   &(data[9]) );       /* rot torque */
    putInt8( sgn(rvel),         &(data[13]) );      /* rot direction */

    RFlexPacket rpacket(MOT_PORT, 0, MOT_AXIS_SET_DIR, 14, data );
    serial.sendPacket(&rpacket);
}

void RFLEX::sendSystemStatusCommand() {
    RFlexPacket packet(SYS_PORT, 0, SYS_STATUS, 0, NULL );
    serial.sendPacket(&packet);
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
            transVelocity = getInt32(&(buffer[19]));
        } else if (axis == 1) {
            bearing = getInt32(&(buffer[15]));
            rotVelocity = getInt32(&(buffer[19]));
        }
        acc       = getInt32(&(buffer[23]));
        trq       = getInt32(&(buffer[27]));
        break;
    default:
        break;
    }
}

//processes a digital io packet from the rflex
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
    if (numIr== 0 && IR_POSES_COUNT > 0) {
        irRanges = new unsigned char[IR_POSES_COUNT];
        if (irRanges != NULL)
            numIr = IR_POSES_COUNT;
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
        for (int i=0; i < IR_BANK_COUNT && ir_data_index < numIr; ++i) {
            for (int j = 0; j < IR_PER_BANK && ir_data_index < numIr; ++j,++ir_data_index) {
                // work out the actual offset in teh packet
                //(current bank + bank offfset) * bank data block size + current ir sensor + constant offset
                int data_index = (IR_BASE_BANK + i) * 13 + j + 11;
                irRanges[ir_data_index] = buffer[data_index];
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
        if (row > lcdY || lcd_length > lcdX) {
            fprintf(stderr,"LCD Data Overflow\n");
            break;
        }

        memcpy(&lcdData[row*lcdX],&buffer[12],lcd_length);

        // if we got whole lcd dump to file
        /*			if (row == 239){
        				FILE * fout;
        				if ((fout = fopen("test.raw","w"))!=0){
        					for (int y=0; y<lcdY; ++y){
        						for (int x=0; x<lcdX;++x){
        							unsigned char Temp = lcdData[y*lcdX + x];
        							for (int i = 0; i < 8; ++i){
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
    int retval, timeStamp, count;
    unsigned char dlen = buffer[5];

    switch (pkt->getOpcode()) {
    case SONAR_REPORT:
        retval    = getInt32(&(buffer[6]));
        timeStamp = getInt32(&(buffer[10]));
        count = 0;
        while ((8+count*3<dlen) && (count<256) && (count < SONAR_MAX_COUNT)) {
            unsigned int sid = buffer[14+count*3];
            sonar_ranges[sid] = getInt16( &(buffer[14+count*3+1]) );
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
            setVelocity((long) (y * JOY_POS_RATIO * ODO_DISTANCE_CONVERSION),
                         (long) (x * JOY_ANG_RATIO * ODO_ANGLE_CONVERSION),
                         (long) (DEFAULT_TRANS_ACCELERATION * ODO_DISTANCE_CONVERSION));
            //RFLEX::joy_control = 5;
        } else if (JoystickWasOn) {
            JoystickWasOn = false;
            setVelocity(0,0,(long) (DEFAULT_TRANS_ACCELERATION * ODO_DISTANCE_CONVERSION));
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

void RFLEX::parsePackets() {
    while (serial.hasPackets()) {
        RFlexPacket* pkt = serial.getPacket();
        parsePacket(pkt);
        delete pkt;
    }
}











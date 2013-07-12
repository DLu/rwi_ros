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
#include <stdio.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <termios.h>
#include <sys/stat.h>
#include <iostream>
#include <unistd.h>
#include <string.h> // memcpy
#include <fcntl.h>

//finds the sign of a value
static long sgn( long val ) {
    if (val < 0)
        return 0;
    else
        return 1;
}

static unsigned int getInt16( const unsigned char *bytes ) {
    unsigned int i;
    memcpy( &i, bytes, 2 );
    return(htons(i));
}


static unsigned long getInt32( const unsigned char *bytes ) {
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
    distance = bearing = transVelocity = rotVelocity = 0;
    voltage = 0;
    offset = 0;
    odomReady = 0;
    found = false;
    brake = true;

    // initialise the LCD dump array
    lcdData=new unsigned char[320*240/8];
    if (lcdData != NULL) {
        lcdX=320/8;
        lcdY=240;
    }
}

int RFLEX::initialize(const char* device_name) {
    // Open the port
    fd = open(device_name, O_RDWR | O_NONBLOCK);
    if (fd == -1) {
        fprintf(stderr,"Could not open serial port %s\n", device_name );
        return -1;
    }

    // Get the terminal info
    struct termios info;
    if (tcgetattr(fd, &info) < 0) {
        fprintf(stderr,"Could not get terminal information for %s\n", device_name );
        return -1;
    }

    // Turn off echo, canonical mode, extended processing, signals, break signal, cr to newline, parity off, 8 bit strip, flow control,
    // size, parity bit, and output processing
    info.c_lflag &= ~(ECHO | ICANON | IEXTEN | ISIG | BRKINT | ICRNL | INPCK | ISTRIP | IXON | CSIZE | PARENB | OPOST);

    // Set size to 8 bits
    info.c_cflag |= CS8;

    // Set time and bytes to enable read at once
    info.c_cc[VTIME] = 0;
    info.c_cc[VMIN] = 0;
    speed_t baud_rate = B115200;
    if (cfsetospeed(&info, baud_rate) < 0) {
        fprintf(stderr,"Could not set the output speed for %s\n", device_name );
        return -1;
    }

    if (cfsetispeed(&info, baud_rate) < 0) {
        fprintf(stderr,"Could not set the input speed for %s\n", device_name );
        return -1;
    }

    // Actually set the controls on the terminal
    if (tcsetattr(fd, TCSAFLUSH, &info) < 0) {
        close(fd);
        fprintf(stderr,"Could not set controls on serial port %s\n", device_name );
    }

    pthread_mutex_init(&writeMutex, NULL);
    pthread_create(&thread, NULL, RFLEX::readThread, this);

    return 0;
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
    sendCommand(SONAR_PORT, 4, SONAR_RUN, 13, data );
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
    sendCommand(IR_PORT, 0, IR_RUN, 21, data );
}

void RFLEX::setBrakePower( const bool on ) {
    int brake;
    if (on)
        brake = MOT_BRAKE_SET;
    else
        brake = MOT_BRAKE_RELEASE;

    sendCommand(MOT_PORT, 0, brake, 0, NULL );
}

void RFLEX::motionSetDefaults(  ) {
    sendCommand(MOT_PORT, 0, MOT_SET_DEFAULTS, 0, NULL );
}

void RFLEX::setDigitalIoPeriod( const long period ) {
    unsigned char data[MAX_COMMAND_LENGTH];
    putInt32( period, &(data[0]) );
    sendCommand(DIO_PORT, 0, DIO_REPORTS_REQ, 4, data );
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
    sendCommand(MOT_PORT, 0, MOT_SYSTEM_REPORT_REQ, 8, data );
}

void RFLEX::setVelocity( const long tvel, const long rvel, const long acceleration) {
    long utvel =labs(tvel);
    long urvel =labs(rvel);
    unsigned char data[MAX_COMMAND_LENGTH];
    // ** workaround for stupid hardware bug, cause unknown, but this works
    // ** with minimal detriment to control
    // ** avoids all values with 1b in highest or 3'rd highest order byte
    
    // 0x1b is part of the packet terminating string
    // which is most likely what causes the bug

    // ** if 2'nd order byte is 1b, round to nearest 1c, or 1a
    if((urvel&0xff00)==0x1b00){
      // ** if lowest order byte is>127 round up, otherwise round down
      urvel=(urvel&0xffff0000)|((urvel&0xff)>127?0x1c00:0x1aff);
    }

    // ** if highest order byte is 1b, round to 1c, otherwise round to 1a
    if((urvel&0xff000000)==0x1b000000){
      // ** if 3'rd order byte is>127 round to 1c, otherwise round to 1a
      urvel=(urvel&0x00ffffff)|(((urvel&0xff0000)>>16)>127?0x1c000000:0x1aff0000);
    }


    putInt8( 0,                 &(data[0]) );       /* forward motion */
    putInt32( utvel,            &(data[1]) );       /* abs trans velocity*/
    putInt32( acceleration,     &(data[5]) );       /* trans acc */
    putInt32( STD_TRANS_TORQUE, &(data[9]) );       /* trans torque */
    putInt8( sgn(tvel),         &(data[13]) );      /* trans direction */

    sendCommand(MOT_PORT, 0, MOT_AXIS_SET_DIR, 14, data );

    putInt8( 1,                 &(data[0]) );       /* rotational motion */
    putInt32( urvel,            &(data[1]) );       /* abs rot velocity  */
    /* 0.275 rad/sec * 10000 */
    putInt32( STD_ROT_ACC,      &(data[5]) );       /* rot acc */
    putInt32( STD_ROT_TORQUE,   &(data[9]) );       /* rot torque */
    putInt8( sgn(rvel),         &(data[13]) );      /* rot direction */

    sendCommand(MOT_PORT, 0, MOT_AXIS_SET_DIR, 14, data );
}

void RFLEX::sendSystemStatusCommand() {
    sendCommand(SYS_PORT, 0, SYS_STATUS, 0, NULL );
}

void RFLEX::parseMotReport( const unsigned char* buffer ) {
    int rv, timeStamp, acc, trq;
    unsigned char axis;

    switch (buffer[PACKET_OPCODE_BYTE]) {
    case MOT_SYSTEM_REPORT:
        rv        = getInt32(&(buffer[6]));
        timeStamp = getInt32(&(buffer[10]));
        axis      = buffer[14];
        if (axis == 0) {
            distance = getInt32(&(buffer[15]));
            transVelocity = getInt32(&(buffer[19]));
            odomReady = odomReady | 1;
        } else if (axis == 1) {
            bearing = getInt32(&(buffer[15]));
            rotVelocity = getInt32(&(buffer[19]));
            odomReady = odomReady | 2;
        }
        acc       = getInt32(&(buffer[23]));
        trq       = getInt32(&(buffer[27]));
        break;
    default:
        break;
    }
}

//processes a digital io packet from the rflex
void RFLEX::parseDioReport( const unsigned char* buffer ) {
    unsigned long timeStamp;
    unsigned char length, address;
    unsigned short data;
    length = buffer[5];

    switch (buffer[PACKET_OPCODE_BYTE]) {
    case DIO_REPORT:
        if (length < 6) {
            fprintf(stderr, "DIO Data Packet too small\n");
            break;
        }
        timeStamp = getInt32(&(buffer[6]));
        address = buffer[10];
        data = getInt16(&(buffer[11]));
        processDioEvent(address, data);

        break;
    default:
        break;
    }
}

void RFLEX::processDioEvent(unsigned char address, unsigned short data) {
    printf("DIO: address 0x%02x (%d) value 0x%02x (%d)\n", address, address, data, data);
}

// Processes the IR sensor report
void RFLEX::parseIrReport( const unsigned char* buffer ) {
    /*
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

    switch (buffer[PACKET_OPCODE_BYTE]) {
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
    /** @todo implement IR Reporting */
}


//processes a sys packet from the rflex - and saves the data in the
//struct for later use, sys is primarily used for bat voltage & brake status
void RFLEX::parseSysReport( const unsigned char* buffer ) {
    unsigned long timeStamp;
    unsigned char length = buffer[5];


    switch (buffer[PACKET_OPCODE_BYTE]) {
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
void RFLEX::parseSonarReport( const unsigned char* buffer ) {
    int retval, timeStamp, count;
    unsigned char dlen = buffer[5];

    switch (buffer[PACKET_OPCODE_BYTE]) {
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
void RFLEX::parseJoyReport( const unsigned char* buffer ) {
    /*unsigned char* buffer = buffer->data();

    static bool JoystickWasOn = false;

    int x,y;
    unsigned char buttons, dlen = buffer[5];

    switch (buffer[PACKET_OPCODE_BYTE]) {
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
    /** @todo implement joystick control */
}


//parses a packet from the rflex, and decides what to do with it
void RFLEX::parsePacket( const unsigned char* buffer ) {
    switch (buffer[PACKET_PORT_BYTE]) {
    case SYS_PORT:
        parseSysReport( buffer );
        break;
    case MOT_PORT:
        parseMotReport( buffer );
        break;
    case JSTK_PORT:
        parseJoyReport( buffer );
        break;
    case SONAR_PORT:
        parseSonarReport( buffer );
        break;
    case DIO_PORT:
        parseDioReport( buffer );
        break;
    case IR_PORT:
        parseIrReport( buffer);
        break;
    default:
        break;
    }
}







void* RFLEX::readThread(void *ptr) {
    RFLEX *rflex = static_cast<RFLEX *>(ptr);

    while (rflex->fd>=0) {
        // Set up the read set to include the serial port
        fd_set read_set;
        FD_ZERO(&read_set);
        FD_SET(rflex->fd, &read_set);

        // Is there any new data to be read from the rFlex?
        if (select(rflex->fd + 1, &read_set, NULL, NULL, NULL) < 0) {
            //debug("Error in select\n");
        } else if (FD_ISSET(rflex->fd, &read_set)) {
            rflex->readPacket();
        }
    }
    return NULL;
}


void RFLEX::readPacket() {
    // If there's no packet ready, just return
    const int read_size = readData();
    if (read_size == 0)
        return;

    // Check to make sure that the packet is the correct size
    const int data_size = read_size - PROTOCOL_SIZE;
    if (readBuffer[PACKET_SIZE_BYTE] != data_size) {
        //debug("Error in packet size.  Expected %i, got %i\n", data_size, static_cast<int>(readBuffer[PACKET_SIZE_BYTE]));
        for (int i = 0; i < BUFFER_SIZE; ++i) {
            //			debug("%2x ", static_cast<int>(readBuffer[i]));
            //			if (((i + 1) % 10) == 0)
            //				debug("\n");
        }
    }

    // Calculate the packet CRC and verify that it matches
    if (computeCRC(readBuffer + PACKET_CRC_START, data_size + PACKET_CRC_OFFSET) != readBuffer[data_size + PACKET_DATA_START_BYTE]) {
        //		debug("CRC error: Expected %i, got %i\n", static_cast<int>(readBuffer[read_size - PROTOCOL_SIZE + PACKET_DATA_START_BYTE]), static_cast<int>(computeCRC(readBuffer + PACKET_CRC_START, data_size + PACKET_CRC_OFFSET)));

        for (int i = 0; i < BUFFER_SIZE; ++i) {
            //			debug("%2x ", static_cast<int>(readBuffer[i]));
            //	if (((i + 1) % 10) == 0)
            //	debug("\n");
        }

        // Eat everything up to the end of the packet
        unsigned char tdata = 0;
        while (tdata != ETX)
            while (read(fd, &tdata, 1) != 1) { }

        return;
    }

    parsePacket(readBuffer);
}


int RFLEX::readData() {
    // Read one byte of of the packet.  No need to check for errors, since this will be called repeatedly.

    int bRead = read(fd, readBuffer + offset, 1);
    if (bRead == 0)
        return 0;
    else if (bRead < 0) {
        printf("Error reading from port!\n");
        return 0;
    }


    // Have we started a packet yet?
    if (!found) {
        // If the first character isn't an ESC, the packet is invalid.  Reset the offset and return.  This
        // will eat badly-formed packets.
        if (readBuffer[0] != ESC) {
            offset = 0;
            return 0;
        }
        if (offset == 0) {
            offset = 1;
            return 0;
        }

        // We have to wait for a STX to show up before it's a valid packet.  If we see an ESC, then we just
        // keep looking for an STX.  If we see something else, give up and start looking for a new packet.
        if (readBuffer[1] == STX) {
            found = true;
            offset = 2;
            return 0;
        } else if (readBuffer[1] == ESC) {
            offset = 1;
            return 0;
        } else {
            offset = 0;
            return 0;
        }
    } else {
        // If the previous character was an ESC,
        if (readBuffer[offset - 1] == ESC) {
            switch (readBuffer[offset]) {
            case NUL:  // Skip over NULs
                read(fd, readBuffer + offset, 1);  // Should we be checking the return code here?
                ++offset;
                return 0;
            case SOH:  // Ignore SOHs by deleting them
                --offset;
                return 0;
            case ETX: // ETX ends the packet, so return the length
                const int retval = offset + 1;
                found = false;
                offset = 0;
                return retval;
            };
        } else {
            // Just increment the counter
            ++offset;

            return 0;
        }
    }

    // Should never get here
    return 0;
}


bool RFLEX::sendCommand(const unsigned char port, const unsigned char id, const unsigned char opcode, const int length, unsigned char* data) {
    pthread_mutex_lock(&writeMutex);

    // Header
    writeBuffer[0] = ESC;
    writeBuffer[1] = STX;
    writeBuffer[PACKET_PORT_BYTE] = port;
    writeBuffer[PACKET_ID_BYTE] = id;
    writeBuffer[PACKET_OPCODE_BYTE] = opcode;
    writeBuffer[PACKET_SIZE_BYTE] = static_cast<unsigned char>(length);
    for (int i=0; i<length; i++) {
        writeBuffer[6+i] = data[i];
    }
    // Footer
    writeBuffer[length + PACKET_DATA_START_BYTE] = computeCRC(writeBuffer + PACKET_CRC_START, length + PACKET_CRC_OFFSET);
    writeBuffer[length + PACKET_DATA_START_BYTE + 1] = ESC;
    writeBuffer[length + PACKET_DATA_START_BYTE + 2] = ETX;

    int ret = writePacket(length + 9);
    pthread_mutex_unlock(&writeMutex);
    return ret;
}


bool RFLEX::writePacket(const int length) const {
    if (fd<0)
        return false;

    int bytes_written = 0;

    while (bytes_written < length) {
        int n = write(fd, writeBuffer + bytes_written, length - bytes_written);
        if (n < 0)
            return false;
        else
            bytes_written += n;

        // Put in a short wait to let the rFlex controller catch up
        usleep(1000);
    }

    return true;
}

unsigned char RFLEX::computeCRC(const unsigned char *buffer, const int n) {
    int crc =buffer[0];
    for (int i = 1; i < n; ++i)
        crc ^= buffer[i];
    return crc;
}


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

#include <sys/types.h>
#include <sys/time.h>
#include <netinet/in.h>

#include <rflex/rflex_info.h>
#include <rflex/rflex_commands.h>
#include <rflex/rflex_io.h>
#include <rflex/rflex_configs.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

//finds the sign of a value
static int sgn( long val ) {
    if (val<0) {
        return(0);
    } else {
        return(1);
    }
}

/* COMPUTE CRC CODE */

static int computeCRC( unsigned char *buf, int nChars ) {
    int i, crc;
    if (nChars==0) {
        crc = 0;
    } else {
        crc = buf[0];
        for (i=1; i<nChars; i++) {
            crc ^= buf[i];
        }
    }
    return(crc);
}

/* CONVERSION BYTES -> NUM */

#if 0

static unsigned int convertBytes2UInt8( unsigned char *bytes ) {
    unsigned int i;
    memcpy( &i, bytes, 1 );
    return(i);
}

#endif

static unsigned int convertBytes2UInt16( unsigned char *bytes ) {
    unsigned int i;
    memcpy( &i, bytes, 2 );
    return(htons(i));
}


static unsigned long convertBytes2UInt32( unsigned char *bytes ) {
    unsigned long i;
    memcpy( &i, bytes, 4 );
    return(htonl(i));
}

/* CONVERSION NUM -> BYTES */

static void convertUInt8( unsigned int i, unsigned char *bytes ) {
    memcpy( bytes, &i, 1 );
}

#if 0

static void convertUInt16( unsigned int i, unsigned char *bytes ) {
    uint16_t conv;
    conv = htonl( i );
    memcpy( bytes, &conv, 2 );
}

#endif

static void convertUInt32( unsigned long l, unsigned char *bytes ) {
    uint32_t conv;
    conv = htonl( l );
    memcpy( bytes, &conv, 4 );
}

//sends a command to the rflex
void RFLEX::cmdSend( int port, int id, int opcode, int len, unsigned char *data ) {
    int i;
    static unsigned char cmd[MAX_COMMAND_LENGTH];
    /* START CODE */
    cmd[0] = 0x1b;
    cmd[1] = 0x02;
    /* PORT */
    cmd[2] = (unsigned char) port;
    /* ID */
    cmd[3] = (unsigned char) id;
    /* OPCODE */
    cmd[4] = (unsigned char) opcode;
    /* LENGTH */
    cmd[5] = (unsigned char) len;
    /* DATA */
    for (i=0; i<len; i++) {
        cmd[6+i] = data[i];
    }
    /* CRC */
    cmd[6+len] = computeCRC( &(cmd[2]), len+4 );    /* END CODE */
    cmd[6+len+1] = 0x1b;
    cmd[6+len+2] = 0x03;

    //pthread_testcancel();
    writeData( fd, cmd, 9+len );

    // Some issues with commands not being recognised if sent too rapidly
    // (too small a buffer on recieving end?
    // So we delay for a bit, specifically we wait until specified amount of
    // time has passed without recieving a packet. This roughtly approximates
    // to waiting till the command has finshed being executed on the robot
    int count;
    timeval now;
    timeval start = {0,0};
    do {
        count = clear_incoming_data();
        gettimeofday(&now,NULL);
        if (count > 0 )
            start = now;
        count = (now.tv_sec - start.tv_sec) * 1000000 + (now.tv_usec - start.tv_usec);

        // release somewhat so other threads can run.
        usleep(500);
    } while (count < 10000);


    //pthread_testcancel();
}

void RFLEX::sonars_on() {
    unsigned char data[MAX_COMMAND_LENGTH];
    convertUInt32( (unsigned long) SONAR_ECHO_DELAY, &(data[0]) );
    convertUInt32( (unsigned long) SONAR_PING_DELAY, &(data[4]) );
    convertUInt32( (unsigned long) SONAR_SET_DELAY , &(data[8]) );
    convertUInt8(  (unsigned int) 2, &(data[12]) );
    cmdSend(SONAR_PORT, 4, SONAR_RUN, 13, data );
}

void RFLEX::sonars_off(  ) {
    unsigned char data[MAX_COMMAND_LENGTH];
    convertUInt32( (long) 0, &(data[0]) );
    convertUInt32( (long) 0, &(data[4]) );
    convertUInt32( (long) 0, &(data[8]) );
    convertUInt8(  (int) 0, &(data[12]) );
    cmdSend(SONAR_PORT, 4, SONAR_RUN, 13, data );
}

void RFLEX::digital_io_on( int period ) {
    unsigned char data[MAX_COMMAND_LENGTH];
    convertUInt32( (long) period, &(data[0]) );
    cmdSend(DIO_PORT, 0, DIO_REPORTS_REQ, 4, data );
}

void RFLEX::digital_io_off(  ) {
    unsigned char data[MAX_COMMAND_LENGTH];
    convertUInt32( (long) 0, &(data[0]) );
    cmdSend(DIO_PORT, 4, DIO_REPORTS_REQ, 4, data );
}

void RFLEX::ir_on(  ) {
    unsigned char data[MAX_COMMAND_LENGTH];
    convertUInt32( (long) 0, &(data[0]) );
    convertUInt32( (long) 70, &(data[4]) );
    convertUInt32( (long) 10, &(data[8]) );
    convertUInt32( (long) 20, &(data[12]) );
    convertUInt32( (long) 150, &(data[16]) );
    convertUInt8( (unsigned char) 2, &(data[20]) );
    cmdSend(IR_PORT, 0, IR_RUN, 21, data );
}

void RFLEX::ir_off(  ) {
    unsigned char data[MAX_COMMAND_LENGTH];
    convertUInt32( (long) 0, &(data[0]) );
    convertUInt32( (long) 0, &(data[4]) );
    convertUInt32( (long) 0, &(data[8]) );
    convertUInt32( (long) 0, &(data[12]) );
    convertUInt32( (long) 0, &(data[16]) );
    convertUInt8( (unsigned char) 0, &(data[20]) );
    cmdSend(IR_PORT, 0, IR_RUN, 21, data );
}
void RFLEX::brake_on(  ) {
    cmdSend(MOT_PORT, 0, MOT_BRAKE_SET, 0, NULL );
}

void RFLEX::brake_off(  ) {
    cmdSend(MOT_PORT, 0, MOT_BRAKE_RELEASE, 0, NULL );
}

void RFLEX::motion_set_defaults(  ) {
    cmdSend(MOT_PORT, 0, MOT_SET_DEFAULTS, 0, NULL );
}

void RFLEX::odometry_on( long period ) {
    unsigned char data[MAX_COMMAND_LENGTH];
    convertUInt32( period, &(data[0]) );         /* period in ms */
    convertUInt32( (long) 3, &(data[4]) );       /* mask */
    cmdSend(MOT_PORT, 0, MOT_SYSTEM_REPORT_REQ, 8, data );
}

void RFLEX::odometry_off(  ) {
    unsigned char data[MAX_COMMAND_LENGTH];
    convertUInt32( (long) 0, &(data[0]) );       /* period in ms */
    convertUInt32( (long) 0, &(data[4]) );       /* mask */
    cmdSend(MOT_PORT, 0, MOT_SYSTEM_REPORT_REQ, 8, data );
}


void RFLEX::set_velocity( float tvelf, float rvelf,
                          float accelerationf ) {
    unsigned char data[MAX_COMMAND_LENGTH];

    
    long tvel = tvelf * ODO_DISTANCE_CONVERSION;
    long rvel = rvelf * ODO_ANGLE_CONVERSION;
    long acceleration = accelerationf * ODO_DISTANCE_CONVERSION;

    long utvel;
    long urvel;

    utvel=labs(tvel);
    urvel=labs(rvel);

    // ** workaround for stupid hardware bug, cause unknown, but this works
    // ** with minimal detriment to control
    // ** avoids all values with 1b in highest or 3'rd highest order byte

    // 0x1b is part of the packet terminating string
    // which is most likely what causes the bug

    // ** if 2'nd order byte is 1b, round to nearest 1c, or 1a
    if ((urvel&0xff00)==0x1b00) {
        // ** if lowest order byte is>127 round up, otherwise round down
        urvel=(urvel&0xffff0000)|((urvel&0xff)>127?0x1c00:0x1aff);
    }

    // ** if highest order byte is 1b, round to 1c, otherwise round to 1a
    if ((urvel&0xff000000)==0x1b000000) {
        // ** if 3'rd order byte is>127 round to 1c, otherwise round to 1a
        urvel=(urvel&0x00ffffff)|(((urvel&0xff0000)>>16)>127?0x1c000000:0x1aff0000);
    }

    convertUInt8( (long) 0,                 &(data[0]) );       /* forward motion */
    convertUInt32( (long) labs(utvel),        &(data[1]) );       /* abs trans velocity*/
    convertUInt32( (long) acceleration,    &(data[5]) );       /* trans acc */
    convertUInt32( (long) STD_TRANS_TORQUE, &(data[9]) );       /* trans torque */
    convertUInt8( (long) sgn(tvel),         &(data[13]) );      /* trans direction */

    cmdSend(MOT_PORT, 0, MOT_AXIS_SET_DIR, 14, data );

    convertUInt8( (long) 1,                 &(data[0]) );       /* rotational motion */
    convertUInt32( (long) labs(urvel),        &(data[1]) );       /* abs rot velocity  */
    /* 0.275 rad/sec * 10000 */
    convertUInt32( (long) STD_ROT_ACC,      &(data[5]) );       /* rot acc */
    convertUInt32( (long) STD_ROT_TORQUE,   &(data[9]) );       /* rot torque */
    convertUInt8( (long) sgn(rvel),         &(data[13]) );      /* rot direction */

    cmdSend(MOT_PORT, 0, MOT_AXIS_SET_DIR, 14, data );
}

void RFLEX::stop_robot( int deceleration) {
    set_velocity( 0, 0, deceleration);
}

int RFLEX::open_connection(const char *device_name) {
    RFLEX_Device   rdev;

    strncpy( rdev.ttyport, device_name, MAX_NAME_LENGTH);
    rdev.baud           = 115200;
    rdev.databits       = 8;
    rdev.parity         = N;
    rdev.stopbits       = 1;
    rdev.hwf            = 0;
    rdev.swf            = 0;

    printf("trying port %s\n",rdev.ttyport);
    if (DEVICE_connect_port( &rdev )<0) {
        fprintf(stderr,"Can't open device %s\n",rdev.ttyport);
        return -1;
    }

    fd = rdev.fd;
    odometry_on(100000);
    digital_io_on( 100000);
    motion_set_defaults();
    return 0;
}

int RFLEX::close_connection() {
    if (fd < 0)
        return -1;
    motion_set_defaults();
    odometry_off();
    digital_io_off();
    sonars_off();
    ir_off();

    printf("Closing rflex serial port\n");
    close(fd);
    fd=-1;

    return 0;
}


//processes a motor packet from the rflex - and saves the data in the
//struct for later use
void RFLEX::parseMotReport( unsigned char *buffer ) {
    int rv, timeStamp, acc, trq;
    unsigned char axis, opcode;

    opcode = buffer[4];
    switch (opcode) {
    case MOT_SYSTEM_REPORT:
        rv        = convertBytes2UInt32(&(buffer[6]));
        timeStamp = convertBytes2UInt32(&(buffer[10]));
        axis      = buffer[14];
        if (axis == 0) {
            distance = convertBytes2UInt32(&(buffer[15]));
            t_vel = convertBytes2UInt32(&(buffer[19]));
        } else if (axis == 1) {
            bearing = convertBytes2UInt32(&(buffer[15]));
            r_vel = convertBytes2UInt32(&(buffer[19]));
        }
        acc       = convertBytes2UInt32(&(buffer[23]));
        trq       = convertBytes2UInt32(&(buffer[27]));
        break;
    default:
        break;
    }
}

//processes a dio packet from the rflex - and saves the data in the
//struct for later use, dio report includes bump sensors...
void RFLEX::parseDioReport( unsigned char *buffer ) {
    unsigned long timeStamp;
    unsigned char opcode, length, address;
    unsigned short data;

    opcode = buffer[4];
    length = buffer[5];


    switch (opcode) {
    case DIO_REPORT:
        if (length < 6) {
            fprintf(stderr, "DIO Data Packet too small\n");
            break;
        }
        timeStamp = convertBytes2UInt32(&(buffer[6]));
        address = buffer[10];
        data = convertBytes2UInt16(&(buffer[11]));

        // Check for the heading home event;
        if (HEADING_HOME_ADDRESS == address) {
            if (home_bearing_found)
                break;
            static bool found_first = false;
            static int first_home_bearing = 0;
            if (found_first) {
                if ((first_home_bearing - bearing) > 0.785* ODO_ANGLE_CONVERSION) {
                    first_home_bearing=static_cast<int> (first_home_bearing-ODO_ANGLE_CONVERSION*2*M_PI);
                } else if ((first_home_bearing - bearing) < 0.785* ODO_ANGLE_CONVERSION) {
                    first_home_bearing=static_cast<int> (first_home_bearing+ODO_ANGLE_CONVERSION*2*M_PI);
                }
                if (abs(first_home_bearing - bearing) > 0.01 * ODO_ANGLE_CONVERSION) {
                    home_bearing=bearing > first_home_bearing? bearing:first_home_bearing;
                    home_bearing_found = true;
                    printf("Home bearing found %d\n",home_bearing);
                }
            } else {
                first_home_bearing=bearing;
                found_first = true;
            }
            break;
        }


        if (BUMPER_STYLE == BUMPER_ADDRESS_STYLE) {
            // on the b21r the bump packets are address 0x40 -> 0x4D, there are some other dio packets
            // but dont know what they do so we throw them away

            // check if the dio packet came from a bumper packet
            if ((address < BUMPER_ADDRESS) || (address >= (BUMPER_ADDRESS+num_bumpers))) {
                // not bumper
                fprintf(stderr,"(dio) address = 0x%02x ",address);
                break;
            } else {
                // is bumper
                //fprintf(stderr,"(bump) address = 0x%02x ",address);
                // assign low data byte to the bumpers (16 bit DIO data, low 4 bits give which corners or the panel are 'bumped')
                bumpers[address - BUMPER_ADDRESS] = data & 0x0F;
            }
        } else {
            // on the magellan pro the bump packets are address 0x40 and 0x41. Each bits of these address
            // match one bumper

            // Check if the dio paquet came from a bumper packet
            if ((address < BUMPER_ADDRESS) || (address >= (BUMPER_ADDRESS+(num_bumpers/8)))) {
                // not bumper
                fprintf(stderr,"(dio) address = 0x%02x ",address);
                break;
            } else {
                // is bumper
                fprintf(stderr,"(bump) address = 0x%02x ",address);

                // Loop for each bit
                for (int i=0; i<8; i++) {
                    // assign each bit of the data to a bumper.
                    bumpers[((address - BUMPER_ADDRESS) * 8 )+ i] = data & (0x01 << i);
                }
            }
        }
        break;
    default:
        break;
    }
}

// Processes the IR sensor report
void RFLEX::parseIrReport( unsigned char *buffer ) {
    // unsigned long timeStamp;
    unsigned char opcode, length;//, address;
    //unsigned short data;

    opcode = buffer[4];
    length = buffer[5];

//	for (int i = 0; i < length; ++i)
    //	printf("%02x",buffer[i+6]);
    //printf("\n");


    // allocate ir storage if we havent already
    // must be a better place to do this but it works
    if (num_ir== 0 && IR_POSES_COUNT > 0) {
        ir_ranges = new unsigned char[IR_POSES_COUNT];
        if (ir_ranges != NULL)
            num_ir = IR_POSES_COUNT;
        else
            fprintf(stderr,"Error allocating ir range storage in rflex status\n");
    }

    switch (opcode) {
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
    }

    // debug print
    /*   for (int i = 0; i < num_ir ; ++i)
       	printf("%02x ", ir_ranges[i]);
    	printf("\n");*/
}


//processes a sys packet from the rflex - and saves the data in the
//struct for later use, sys is primarily used for bat voltage & brake status
void RFLEX::parseSysReport( unsigned char *buffer ) {
    unsigned long timeStamp;
    unsigned char opcode, length;


    opcode = buffer[4];
    length = buffer[5];


    switch (opcode) {
    case SYS_LCD_DUMP:
        // currently designed for 320x240 screen on b21r
        // stored in packed format
        if (length < 6) {
            fprintf(stderr, "Got bad Sys packet (lcd)\n");
            break;
        }

        unsigned char lcd_length, row;
        timeStamp=convertBytes2UInt32(&(buffer[6]));
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
        timeStamp=convertBytes2UInt32(&(buffer[6]));
        // raw voltage measurement...needs calibration offset added
        voltage=convertBytes2UInt32(&(buffer[10]));
        brake=buffer[14];

        break;

    default:
        fprintf(stderr,"Unknown sys opcode recieved\n");
    }
}

//processes a sonar packet fromt the rflex, and saves the data in the
//struct for later use
//HACK - also buffers data and filters for smallest in last AGE readings
void RFLEX::parseSonarReport( unsigned char *buffer ) {

    unsigned int sid;

    int x,smallest;

    int count, retval, timeStamp;
    unsigned char opcode, dlen;

    opcode = buffer[4];
    dlen   = buffer[5];

    switch (opcode) {
    case SONAR_REPORT:
        if (ranges == NULL || oldranges == NULL)
            return;
        retval    = convertBytes2UInt32(&(buffer[6]));
        timeStamp = convertBytes2UInt32(&(buffer[10]));
        count = 0;
        while ((8+count*3<dlen) && (count<256) && (count < SONAR_COUNT)) {
            sid = buffer[14+count*3];
            //shift buffer
            for (x=0;x<SONAR_AGE-1;x++)
                oldranges[x+1+sid*SONAR_AGE]=oldranges[x+sid*SONAR_AGE];
            //add value to buffer
            smallest=oldranges[0+sid*SONAR_AGE]=convertBytes2UInt16( &(buffer[14+count*3+1]));
            //find the smallest
            for (x=1;x<SONAR_AGE;x++)
                if (smallest>oldranges[x+sid*SONAR_AGE])
                    smallest=oldranges[x+sid*SONAR_AGE];
            //set the smallest in last sonar_age as our value
            ranges[sid] = smallest;
            count++;
        }
        break;
    default:
        break;
    }
}

//processes a joystick packet fromt the rflex, and sets as command if
// joystick command enabled
void RFLEX::parseJoyReport( unsigned char *buffer ) {
    static bool JoystickWasOn = false;

    int x,y;
    unsigned char opcode, dlen, buttons;

    opcode = buffer[4];
    dlen   = buffer[5];

    switch (opcode) {
    case JSTK_GET_STATE:
        if (dlen < 13) {
            fprintf(stderr,"Joystick Packet too small\n");
            break;
        }
        x = convertBytes2UInt32(&buffer[10]);
        y = convertBytes2UInt32(&buffer[14]);
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
    }
}


//parses a packet from the rflex, and decides what to do with it
int RFLEX::parseBuffer( unsigned char *buffer, unsigned int len ) {
    unsigned int port, dlen, crc;

    port   = buffer[2];
    dlen   = buffer[5];

    if (dlen+8>len) {
        return(0);
    } else {
        crc    = computeCRC( &(buffer[2]), dlen+4 );
        if (crc != buffer[len-3])
            return(0);
        switch (port) {
        case SYS_PORT:
//      fprintf( stderr, "(sys)" );
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
            //fprintf( stderr, "(dio)" );
            parseDioReport( buffer );
            break;
        case IR_PORT:
            parseIrReport( buffer);
//      fprintf( stderr, "(ir)" );
            break;
        default:
            break;
        }
    }
    return(1);
}

// returns number of commands parsed
int RFLEX::clear_incoming_data() {
    unsigned char buffer[4096];
    int len;
    int bytes;
    int count = 0;

    // 32 bytes here because the motion packet is 34. No sense in waiting for a
    // complete packet -- we can starve because the last 2 bytes might always
    // arrive with the next packet also, and we never leave the loop.

    while ((bytes = bytesWaiting(fd)) > 32) {
        count ++;
        //pthread_testcancel();
        waitForAnswer(fd, buffer, &len);
        //pthread_testcancel();
        parseBuffer(buffer, len);
    }
    return count;
}

//returns the odometry data saved in the struct
void RFLEX::update_status(float *distance_o,  float *bearing_o,
                          float *t_vel_o, float *r_vel_o) {
    clear_incoming_data();

    *distance_o = distance / (float) ODO_DISTANCE_CONVERSION;
    if (home_bearing_found)
        *bearing_o = bearing-home_bearing;
    else
        *bearing_o = bearing;
    *bearing_o /= (float) ODO_ANGLE_CONVERSION;
    *t_vel_o = t_vel / (float) ODO_DISTANCE_CONVERSION;
    *r_vel_o = r_vel / (float) ODO_ANGLE_CONVERSION;
}

/* gets actual data and returns it in ranges
 * NOTE - actual mappings are strange
 * each module's sonar are numbered 0-15
 * thus for 4 sonar modules id's are 0-64
 * even if only say 5 or 8 sonar are connected to a given module
 * thus, we record values as they come in
 * (data comes in in sets of n modules, 3 if you have 3 modules for example)
 *
 * note the remmapping done using the config parameters into 0-n, this mapping
 * should come out the same as the mapping adverstised on your robot
 * this if you put the poses in that order in the config file - everything
 * will line up nicely
 *
 * -1 is returned if we get back fewer sonar than requested
 * (meaning your parameters are probobly incorrect) otherwise we return 0
 */
int RFLEX::update_sonar(float** rings) {
    int x,y=0;
    int ringi=0, i=0;
    int total = 0;

    clear_incoming_data();

    //copy all data
    for (x=0;x<SONAR_NUM_BANKS;x++)
        for (y=0;y<SONARS_PER_BANK[x];y++) {
            int range = ranges[x*SONAR_MAX_PER_BANK+y];
            if (range > SONAR_MAX_RANGE)
                range = SONAR_MAX_RANGE;
            float fRange = range / (float) RANGE_CONVERSION;
            rings[ringi][i] = fRange;
            i++;
            if (i>=SONARS_PER_RING[ringi]) {
                i = 0;
                ringi++;
            }
	    total++;
        }
    if (total<SONAR_COUNT) {
        fprintf(stderr,"Requested %d sonar only %d supported\n",SONAR_COUNT,y);
        return -1;
    }
    return 0;
}

// copies data from internal bumper list to the proper rflex bumper list
void RFLEX::update_bumpers( int num_bumpers, char *values) {
    clear_incoming_data();
    // allocate bumper storage if we havent already
    // must be a better place to do this but it works
    // *** watch out this is duplicated ***
    /*	if (num_bumpers != BUMPER_COUNT)
    	{
       		delete bumpers;
    		bumpers = new char[BUMPER_COUNT];
    		if (bumpers != NULL)
    			num_bumpers = BUMPER_COUNT;
    	}*/

    if (num_bumpers > num_bumpers) {
        fprintf(stderr,"Requested more bumpers than available.\n");
        num_bumpers = num_bumpers;
    }

    memcpy(values, bumpers, num_bumpers*sizeof(char));
}


// copies data from internal bumper list to the proper rflex bumper list
void RFLEX::update_ir( int num_irs,
                       unsigned char *values) {
    clear_incoming_data();

    if (num_irs > num_ir) {
        //fprintf(stderr,"Requested more ir readings than available. %d of %d\n", num_irs,num_ir );
        num_irs = num_ir;
    }

    memcpy(values, ir_ranges, num_irs*sizeof(char));
}


//returns the last battery, timestamp and brake information
void RFLEX::update_system(   int *battery,
                             int *brake_o) {
    //cmdSend(SYS_PORT, 0, SYS_LCD_DUMP, 0, NULL );
    cmdSend(SYS_PORT, 0, SYS_STATUS, 0, NULL );
    if (USE_JOYSTICK) {
        cmdSend(JSTK_PORT, 0, JSTK_GET_STATE, 0, NULL);
    }


    clear_incoming_data();

    *battery = voltage;
    //timestamp = timestamp;
    *brake_o = brake;
}



/*
 * same effects are emulated at a higher level - is it possible to
 * do it here?
 */
int RFLEX::initialize(const char* devname) {
    int ret0 = open_connection(devname);
    if (ret0<0) return ret0;

    unsigned char data[MAX_COMMAND_LENGTH];
    int x;

    int trans_acceleration = DEFAULT_TRANS_ACCELERATION / ODO_DISTANCE_CONVERSION;
    int rot_acceleration = DEFAULT_ROT_ACCELERATION / ODO_ANGLE_CONVERSION;

    data[0] = 0;
    convertUInt32( (long) 0, &(data[1]) );          /* velocity */
    convertUInt32( (long) trans_acceleration, &(data[5]) );
    /* acceleration */
    convertUInt32( (long) 0, &(data[9]) );      /* torque */
    data[13] = 0;

    cmdSend(MOT_PORT, 0, MOT_AXIS_SET_DIR, 14, data );

    data[0] = 1;
    convertUInt32( (long) 0, &(data[1]) );          /* velocity */
    convertUInt32( (long) rot_acceleration, &(data[5]) );
    /* acceleration */
    convertUInt32( (long) 0, &(data[9]) );      /* torque */
    data[13] = 0;

    cmdSend(MOT_PORT, 0, MOT_AXIS_SET_DIR, 14, data );

    //mark all non-existant (or no-data) sonar as such
    //note - this varies from MAX_INT set when sonar fail to get a valid echo.
    ranges=(int*) malloc(SONAR_MAX_COUNT*sizeof(int));
    oldranges=(int*) malloc(SONAR_MAX_COUNT*SONAR_AGE*sizeof(int));
    for (x=0;x<SONAR_MAX_COUNT;x++)
        ranges[x]=-1;

    // initialise the LCD dump array
    lcd_data=new unsigned char[320*240/8];
    if (lcd_data != NULL) {
        lcd_x=320/8;
        lcd_y=240;
    }

    // allocate bumper storage if we havent already
    // must be a better place to do this but it works
    // *** watch out this is duplicated ***
    if (num_bumpers != BUMPER_COUNT) {
        delete bumpers;
        bumpers = new char[BUMPER_COUNT];
        if (bumpers != NULL)
            num_bumpers = BUMPER_COUNT;
        for (int i = 0; i < num_bumpers; ++i)
            bumpers[i] = 0;
    }
    home_bearing_found=true;
    home_bearing = -967561;
    return 0;
}













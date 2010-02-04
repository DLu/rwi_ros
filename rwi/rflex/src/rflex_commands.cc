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
#include <rflex/rflex_configs.h>


#include <rflex/rflex_commands.h>
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

void RFLEX::setSonarPower(bool on) {
    unsigned long echo = on?SONAR_ECHO_DELAY:0,
                         ping = on?SONAR_PING_DELAY:0,
                                set  = on?SONAR_SET_DELAY:0,
                                       val  = on?2:0;

    unsigned char data[MAX_COMMAND_LENGTH];
    putInt32( echo, &(data[0]) );
    putInt32( ping, &(data[4]) );
    putInt32( set , &(data[8]) );
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


void RFLEX::set_velocityF( float tvelf, float rvelf,
                           float accelerationf ) {



    long tvel = tvelf * ODO_DISTANCE_CONVERSION;
    long rvel = rvelf * ODO_ANGLE_CONVERSION;
    long acceleration = accelerationf * ODO_DISTANCE_CONVERSION;
    set_velocity(tvel, rvel, acceleration);
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

void RFLEX::stop_robot( int deceleration) {
    set_velocity( 0, 0, deceleration);
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
void RFLEX::parseIrReport( RFlexPacket* pkt ) {

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
    }
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

//processes a sonar packet fromt the rflex, and saves the data in the
//struct for later use
//HACK - also buffers data and filters for smallest in last AGE readings
void RFLEX::parseSonarReport( RFlexPacket* pkt ) {
    unsigned char* buffer = pkt->data();
    unsigned int sid;
    int x,smallest;

    int count, retval, timeStamp;
    unsigned char dlen = buffer[5];

    switch (pkt->getOpcode()) {
    case SONAR_REPORT:
        if (ranges == NULL || oldranges == NULL)
            return;
        retval    = getInt32(&(buffer[6]));
        timeStamp = getInt32(&(buffer[10]));
        count = 0;
        while ((8+count*3<dlen) && (count<256) && (count < SONAR_COUNT)) {
            sid = buffer[14+count*3];
            //shift buffer
            for (x=0;x<SONAR_AGE-1;x++)
                oldranges[x+1+sid*SONAR_AGE]=oldranges[x+sid*SONAR_AGE];
            //add value to buffer
            smallest=oldranges[0+sid*SONAR_AGE]=getInt16( &(buffer[14+count*3+1]));
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
void RFLEX::parseJoyReport( RFlexPacket* pkt ) {
    unsigned char* buffer = pkt->data();
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
    }
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

//returns the odometry data saved in the struct
void RFLEX::update_status(float *distance_o,  float *bearing_o,
                          float *t_vel_o, float *r_vel_o) {
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
    if (num_irs > num_ir) {
        //fprintf(stderr,"Requested more ir readings than available. %d of %d\n", num_irs,num_ir );
        num_irs = num_ir;
    }

    memcpy(values, ir_ranges, num_irs*sizeof(char));
}


//returns the last battery, timestamp and brake information
void RFLEX::update_system(   int *battery,
                             int *brake_o) {
    //RFlexPacket packet(SYS_PORT, 0, SYS_LCD_DUMP, 0, NULL );
    RFlexPacket packet(SYS_PORT, 0, SYS_STATUS, 0, NULL );
    if (USE_JOYSTICK) {
        RFlexPacket packet(JSTK_PORT, 0, JSTK_GET_STATE, 0, NULL);
    }

    *battery = voltage;
    //timestamp = timestamp;
    *brake_o = brake;
}



int RFLEX::open_connection(const char *device_name) {
    serial = new SerialPort();
    if (serial->open_connection(device_name, 115200)<0)
        return -1;

#warning move outside
    setOdometryPeriod (100000);
    setDigitalIoPeriod(100000);
    motion_set_defaults();
    return 0;
}

int RFLEX::close_connection() {
    motion_set_defaults();
    setOdometryPeriod(0);
    setDigitalIoPeriod(0);
    setSonarPower(false);
    setIrPower(false);
    delete serial;
    return 0;
}


int RFLEX::initialize(const char* devname) {
    int ret0 = open_connection(devname);
    if (ret0<0) return ret0;

    int x;
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


void RFLEX::parsePackets() {
    while (serial->hasPackets()) {
        RFlexPacket* pkt = serial->getPacket();
        parsePacket(pkt);
        delete pkt;
    }
}











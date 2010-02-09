/*
 *  RFLEX Driver Packet - By David Lu!! 2/2010
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

#include<rflex/rflex_packet.h>
#include<string.h>

RFlexPacket::RFlexPacket(unsigned char* buffer, int len) {
    packet = new unsigned char[len];
    memcpy(packet, buffer, len);
    this->len = len;
}

RFlexPacket::RFlexPacket(const int port, const int id, const int opcode, const int len, unsigned char *data) {
    int i;
    packet = new unsigned char[MAX_COMMAND_LENGTH];
    packet[0] = ESC;     /* START CODE */
    packet[1] = STX;
    packet[2] = (unsigned char) port;
    packet[3] = (unsigned char) id;
    packet[4] = (unsigned char) opcode;
    packet[5] = (unsigned char) len;
    for (i=0; i<len; i++) {
        packet[6+i] = data[i];
    }
    packet[6+len] = computeCRC( &(packet[2]), len+4 );    /* END CODE */
    packet[6+len+1] = 0x1b;
    packet[6+len+2] = 0x03;

    this->len = 9+len;
}

RFlexPacket::~RFlexPacket() {
    delete packet;
}

unsigned int RFlexPacket::computeCRC(const unsigned char* data, const int len) const {
    int i, crc;
    if (len==0) {
        crc = 0;
    } else {
        crc = data[0];
        for (i=1; i<len; i++) {
            crc ^= data[i];
        }
    }
    return(crc);
}

bool RFlexPacket::isValid() const {
    unsigned int dlen = packet[5];

    if (dlen+8>len)
        return false;

    int crc = computeCRC( &(packet[2]), dlen+4 );
    if (crc != packet[len-3])
        return false;
    return true;
}






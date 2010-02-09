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
#ifndef RFLEX_PACKET_H
#define RFLEX_PACKET_H

// Escape codes used in the data packets
static const unsigned char NUL = 0;
static const unsigned char SOH = 1;
static const unsigned char STX = 2;
static const unsigned char ETX = 3;
static const unsigned char ESC = 27;
#define MAX_COMMAND_LENGTH             256

class RFlexPacket {
    public:
        RFlexPacket(unsigned char* buffer, int len);
        RFlexPacket(const int port, const int id, const int opcode, const int len, unsigned char *data);
        virtual ~RFlexPacket();

        unsigned int length() const {
            return len;
        }
        unsigned char* data() const {
            return packet;
        }

        bool isValid() const;
        unsigned int getPort() const {
            return packet[2];
        }
        unsigned int getOpcode() const {
            return packet[4];
        }

    private:
        unsigned int computeCRC(const unsigned char* data, const int len) const;
        unsigned char* packet;
        unsigned int len;

        // Not allowed to use these
        RFlexPacket(const RFlexPacket &pkt);
        RFlexPacket &operator=(const RFlexPacket &pkt);
};
#endif

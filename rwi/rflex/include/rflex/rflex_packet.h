#ifndef RFLEX_PACKET_H
#define RFLEX_PACKET_H


#include <netinet/in.h>
#include <string.h>
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
        RFlexPacket(int port, int id, int opcode, int len, unsigned char *data);

        unsigned int length() {
            return len;
        }
        unsigned char* data() {
            return packet;
        }

        bool isValid();
        unsigned int getPort() {
            return packet[2];
        }
        unsigned int getOpcode() {
            return packet[4];
        }

    private:
        unsigned int computeCRC(unsigned char* data, int len);

        unsigned char* packet;
        unsigned int len;
};




#endif

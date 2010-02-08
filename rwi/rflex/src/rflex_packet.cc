#include<rflex/rflex_packet.h>
#include<string.h>

RFlexPacket::RFlexPacket(unsigned char* buffer, int len) {
    packet = new unsigned char[len];
    memcpy(packet, buffer, len);
    this->len = len;
}

RFlexPacket::RFlexPacket(int port, int id, int opcode, int len, unsigned char *data) {
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

unsigned int RFlexPacket::computeCRC(unsigned char* data, int len) {
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

bool RFlexPacket::isValid() {
    unsigned int dlen = packet[5];

    if (dlen+8>len)
        return false;

    int crc = computeCRC( &(packet[2]), dlen+4 );
    if (crc != packet[len-3])
        return false;
    return true;
}






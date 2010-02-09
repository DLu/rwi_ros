#ifndef SERIAL_H
#define SERIAL_H

/*
 *  Serial Port Communication for the RFLEX Driver - By David Lu!! 2/2010
 *
 *  Writes packets through sendPacket interface
 *  Recieved packets are queued
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

#include <termios.h>
#include <fcntl.h>
#include <pthread.h>
#include <queue>
#include <rflex/rflex_packet.h>
#define BUFFER_SIZE 1024

class SerialPort {
    public:
        SerialPort();
        virtual ~SerialPort();

        int openConnection(const char* port, int speed);
        int setBaudRate(const int speed) const;
        int baudRate() const;
        void sendPacket(RFlexPacket* pkt);

        inline bool hasPackets() const {
            return queue.size() >0;
        }
        inline int size() const {
            return queue.size();
        }

        // Returns a pointer to the packet at the front of the queue
        RFlexPacket* getPacket() {
            RFlexPacket* pkt = queue.front();
            queue.pop();
            return pkt;
        }

    private:
        const char* port;
        int fd;
        int speed;
        pthread_t thread;
        bool found;
        int offset;
        unsigned char readBuffer[BUFFER_SIZE];

        static int rate(speed_t baud);
        static speed_t baud(const int speed);

        static void *readThread(void *ptr);
        void readPacket();
        int readData();

        std::queue<RFlexPacket*> queue;
        pthread_mutex_t writeMutex;

        // Not allowed to use these
        SerialPort(const SerialPort &sp);
        SerialPort &operator=(const SerialPort &sp);

};

#endif

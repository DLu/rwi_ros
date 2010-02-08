#ifndef SERIAL_H
#define SERIAL_H

/* Serial Port Communication
 * Writes packets through sendPacket interface
 * Recieved packets are queued
 * David Lu!! - 2/2010
 */

#include <termios.h>
#include <fcntl.h>
#include <pthread.h>
#include <queue>
#include <rflex/rflex_packet.h>
#define BUFFER_SIZE 1024

class SerialPort {
    public:
        ~SerialPort();

        int openConnection(const char* port, int speed);
        int setBaudRate(const int speed) const;
        int baudRate() const;
        void sendPacket(RFlexPacket* pkt);

        inline bool hasPackets() {
            return queue.size() >0;
        }
        inline int size() {
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

};

#endif

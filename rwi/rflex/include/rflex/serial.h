#ifndef SERIAL_H
#define SERIAL_H

#include <termios.h>
#include <fcntl.h>
#include <pthread.h>
#include <queue>
#include <rflex/rflex_packet.h>
#define BUFFER_SIZE 1024

class SerialPort {
    public:
        ~SerialPort();

        int open_connection(const char* port, int speed);
        int setBaudRate(const int speed) const;
        int baudRate() const;
        void sendPacket(RFlexPacket* pkt);

        bool hasPackets() {
            return queue.size() >0;
        }
        int size() {
            return queue.size();
        }
        RFlexPacket* getPacket() {
            RFlexPacket* pkt = queue.front();
            queue.pop();
            return pkt;
        }

    private:
        const char* m_port;
        int m_fd;
        int m_speed;
        pthread_t m_read_thread;
        bool m_found;
        int m_offset;
        unsigned char m_read_buffer[BUFFER_SIZE];

        static int rate(speed_t baud);
        static speed_t baud(const int speed);

        static void *readThread(void *ptr);
        void readPacket();
        int readData();

        std::queue<RFlexPacket*> queue;
        pthread_mutex_t m_write_mutex;

};

#endif

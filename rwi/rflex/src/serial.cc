#include <rflex/serial.h>
#include <rflex/rflex_packet.h>
#include <termios.h>
#include <sys/stat.h>
#include <iostream>
#include<string>
#include <unistd.h>
using namespace std;

// These are the mappings from integer baud rates to the macros used by the system calls that manipulate the termios struct
static const int g_num_rates = 31;
static const speed_t g_baud_rates[g_num_rates] = {
    B0, B50, B75, B110, B134, B150, B200, B300, B600, B1200,
    B1800, B2400, B4800, B9600, B19200, B38400, B57600, B115200, B230400, B460800,
    B500000, B576000, B921600, B1000000, B1152000, B1500000, B2000000, B2500000, B3000000, B3500000,
    B4000000
};
static const int g_speeds[g_num_rates] = {
    0, 50, 75, 110, 134, 150, 200, 300, 600, 1200,
    1800, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800,
    500000, 576000, 921600, 1000000, 1152000, 1500000, 2000000, 2500000, 3000000, 3500000,
    4000000
};

int SerialPort::open_connection(const char* port, const int speed) {
    // Open the port
    m_port = port;
    m_fd = open(port, O_RDWR | O_NONBLOCK);
    if (m_fd == -1) {
        cerr << "Could not open serial port " << port << endl;
        return -1;
    }

    // Get the terminal info
    struct termios info;
    if (tcgetattr(m_fd, &info) < 0) {
        cerr << "Could not get terminal information for " << port << endl;
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

    // Actually set the controls on the terminal
    if (tcsetattr(m_fd, TCSAFLUSH, &info) < 0) {
        close(m_fd);
        cerr << "Could not set controls on serial port " << port << endl;
    }

    setBaudRate(speed);
	
	pthread_mutex_init(&m_write_mutex, NULL);
    pthread_create(&m_read_thread, NULL, SerialPort::readThread, this);

    return 0;
}


SerialPort::~SerialPort() {
    if (m_fd != -1)
        close(m_fd);
}

void* SerialPort::readThread(void *ptr) {
    SerialPort *serial = static_cast<SerialPort *>(ptr);

    while (true) {
        // Set up the read set to include the serial port
        fd_set read_set;
        FD_ZERO(&read_set);
        FD_SET(serial->m_fd, &read_set);

        // Is there any new data to be read from the port?
        if (select(serial->m_fd + 1, &read_set, NULL, NULL, NULL) >= 0 && FD_ISSET(serial->m_fd, &read_set)) {
            int read_size = serial->readData();
            if (read_size > 0) {
                RFlexPacket* pkt = new RFlexPacket(serial->m_read_buffer, read_size);
                serial->queue.push(pkt);
            }
        }
    }
}

int SerialPort::readData() {
    // Read one byte of of the packet.  No need to check for errors, since this will be called repeatedly.
    if (read(m_fd, m_read_buffer + m_offset, 1) != 1)
        return 0;

    // Have we started a packet yet?
    if (!m_found) {
        // If the first character isn't an ESC, the packet is invalid.  Reset the offset and return.  This
        // will eat badly-formed packets.
        if (m_read_buffer[0] != ESC) {
            m_offset = 0;
            return 0;
        }
        if (m_offset == 0) {
            m_offset = 1;
            return 0;
        }

        // We have to wait for a STX to show up before it's a valid packet.  If we see an ESC, then we just
        // keep looking for an STX.  If we see something else, give up and start looking for a new packet.
        if (m_read_buffer[1] == STX) {
            m_found = true;
            m_offset = 2;
            return 0;
        } else if (m_read_buffer[1] == ESC) {
            m_offset = 1;
            return 0;
        } else {
            m_offset = 0;
            return 0;
        }
    } else {
        // If the previous character was an ESC,
        if (m_read_buffer[m_offset - 1] == ESC) {
            switch (m_read_buffer[m_offset]) {
            case NUL:  // Skip over NULs
                read(m_fd, m_read_buffer + m_offset, 1);  // Should we be checking the return code here?
                ++m_offset;
                return 0;
            case SOH:  // Ignore SOHs by deleting them
                --m_offset;
                return 0;
            case ETX: // ETX ends the packet, so return the length
                const int retval = m_offset + 1;
                m_found = false;
                m_offset = 0;
                return retval;
            };
        } else {
            // Just increment the counter
            ++m_offset;

            return 0;
        }
    }

    // Should never get here
    return 0;
}


int SerialPort::setBaudRate(const int speed) const {
    // Set the actual speed to pass into the terminal.
    speed_t baud_rate = baud(speed);

    // Now, get the terminal information and set the speed
    struct termios info;
    if (tcgetattr(m_fd, &info) < 0) {
        cerr << "Could not get terminal information for " << m_port << endl;
        return baudRate();
    }

    if (cfsetospeed(&info, baud_rate) < 0) {
        cerr << "Could not set the output speed for " << m_port << endl;
        return baudRate();
    }

    if (cfsetispeed(&info, baud_rate) < 0) {
        cerr << "Could not set the input speed for " << m_port << endl;
        return baudRate();
    }

    if (tcsetattr(m_fd, TCSAFLUSH, &info) < 0) {
        close(m_fd);
        cerr << "Could not set controls on serial port " << m_port << endl;
    }

    return rate(baud_rate);
}


int SerialPort::baudRate() const {
    // Open the terminal.  We actually need to check the read device in case someone else has sneakily set the speed.
    struct termios info;
    if (tcgetattr(m_fd, &info) < 0) {
        cerr << "Could not get terminal information for " << m_port << endl;
        return 0;
    }

    // Get the speeds
    speed_t ospeed = cfgetospeed(&info);
    speed_t ispeed = cfgetispeed(&info);

    // Ensure that they are the same
    if (ospeed != ispeed) {
        cerr << "Input and output speeds are different for " << m_port << "\n  input: " << rate(ispeed) << "\n  output: " << rate(ospeed) << endl;
        return 0;
    }

    return rate(ospeed);
}


int SerialPort::rate(speed_t baud) {
    for (int i = 0; i < g_num_rates ; ++i)
        if (baud == g_baud_rates[i])
            return g_speeds[i];

    // Should never get here
    return 0;
}


speed_t SerialPort::baud(const int speed) {
    for (int i = g_num_rates - 1; i >= 0; --i)
        if (speed >= g_speeds[i])
            return g_baud_rates[i];

    // Should never get here
    return B0;
}

void SerialPort::sendPacket(RFlexPacket* pkt) {
	pthread_mutex_lock(&m_write_mutex);
    int length = pkt->length();
    unsigned char* data = pkt->data();
    int bytes_written = 0;

    while (bytes_written < length) {
        int n = write(m_fd, data + bytes_written, length - bytes_written);
        if (n < 0){
			pthread_mutex_unlock(&m_write_mutex);
            return;
		}
        else
            bytes_written += n;

        usleep(1000);
    }
	pthread_mutex_unlock(&m_write_mutex);
}


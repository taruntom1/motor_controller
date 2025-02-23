#include "UARTProtocol.hpp"

UARTProtocol::UARTProtocol(const ProtocolConfig &cfg) : config(cfg), serialFd(-1) {}

UARTProtocol::~UARTProtocol()
{
    if (serialFd != -1)
    {
        close(serialFd);
    }
}

bool UARTProtocol::begin()
{
    serialFd = open(config.portName.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (serialFd == -1)
    {
        std::cerr << "Failed to open serial port" << std::endl;
        return false;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(serialFd, &tty) != 0)
    {
        std::cerr << "Error getting serial port attributes" << std::endl;
        return false;
    }

    tty.c_cflag = config.baudRate | config.byteSize | CLOCAL | CREAD;
    tty.c_iflag = IGNPAR;
    tty.c_oflag = 0;
    tty.c_lflag = 0;

    if (config.stopBits == 2)
    {
        tty.c_cflag |= CSTOPB;
    }
    if (config.parity)
    {
        tty.c_cflag |= PARENB;
    }
    else
    {
        tty.c_cflag &= ~PARENB;
    }

    tcflush(serialFd, TCIFLUSH);
    if (tcsetattr(serialFd, TCSANOW, &tty) != 0)
    {
        std::cerr << "Error setting serial port attributes" << std::endl;
        return false;
    }

    return true;
}

void UARTProtocol::sendCommand(uint8_t command)
{
    uint8_t commandArr[2] = {config.header, command};
    write(serialFd, commandArr, 2);
    tcflush(serialFd, TCIFLUSH);
}

void UARTProtocol::sendData(const uint8_t *data, size_t length)
{
    write(serialFd, data, length);
}

#include <sys/select.h>
#include <unistd.h>

bool UARTProtocol::readCommand(uint8_t &command, int timeout)
{
    uint8_t buffer;
    int bytesRead;

    fd_set readfds;
    struct timeval tv;

    FD_ZERO(&readfds);
    FD_SET(serialFd, &readfds);

    // Set timeout (seconds + microseconds)
    tv.tv_sec = timeout / 1000;           // Convert ms to seconds
    tv.tv_usec = (timeout % 1000) * 1000; // Convert remaining ms to microseconds

    // Wait for data to be available within the timeout period
    int result = select(serialFd + 1, &readfds, NULL, NULL, &tv);

    if (result == -1)
    {
        // Error occurred in select()
        return false;
    }
    else if (result == 0)
    {
        // Timeout occurred, no data received
        return false;
    }

    // If select() indicates data is available, proceed with reading
    while ((bytesRead = read(serialFd, &buffer, 1)) > 0)
    {
        if (buffer == config.header)
        {
            if (read(serialFd, &command, 1) > 0)
            {
                return true;
            }
            return false;
        }
    }
    return false;
}

bool UARTProtocol::readData(uint8_t *data, size_t length, int timeout)
{
    fd_set readfds;
    struct timeval tv;

    FD_ZERO(&readfds);
    FD_SET(serialFd, &readfds);

    // Set timeout (seconds + microseconds)
    tv.tv_sec = timeout / 1000;           // Convert ms to seconds
    tv.tv_usec = (timeout % 1000) * 1000; // Convert remaining ms to microseconds

    // Wait for data to be available within the timeout period
    int result = select(serialFd + 1, &readfds, NULL, NULL, &tv);

    if (result == -1)
    {
        // Error occurred in select()
        return false;
    }
    else if (result == 0)
    {
        // Timeout occurred, no data received
        return false;
    }

    // If data is available, attempt to read the specified length
    ssize_t bytesRead = read(serialFd, data, length);
    return bytesRead == (ssize_t)length;
}

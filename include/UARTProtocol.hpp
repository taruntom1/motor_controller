#ifndef UARTPROTOCOL_H
#define UARTPROTOCOL_H

#include <iostream>
#include <iomanip>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <sys/select.h>
#include <cstdint>

struct ProtocolConfig
{
    std::string portName = "/dev/ttyUSB0"; // Change as needed
    speed_t baudRate = B115200;
    uint8_t byteSize = CS8;
    uint8_t stopBits = 1;
    uint8_t parity = 0;
    uint8_t header = 0xAA;
    uint8_t maxPacketSize = 100;
};

class UARTProtocol
{
private:
    int serialFd;
    ProtocolConfig config;

public:
    UARTProtocol(const ProtocolConfig &config);
    ~UARTProtocol();
    bool begin();
    void sendCommand(uint8_t command);
    void sendData(const uint8_t *data, size_t length);
    bool readCommand(uint8_t &command, int timeout = 1000);
    bool readData(uint8_t *data, size_t length, int timeout = 1000);
};

#endif // UARTPROTOCOL_H

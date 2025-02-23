#ifndef COMMANDS_H
#define COMMANDS_H
#include <stdint.h>

const uint8_t PING = 0x01;
const uint8_t READ_SUCCESS = 0x02;
const uint8_t READ_FAILURE = 0x03;
const uint8_t START = 0x04;
const uint8_t STOP = 0x05;
const uint8_t RESET = 0x06;

const uint8_t SET_MOTOR_DATA = 0x10;
const uint8_t GET_MOTOR_DATA = 0x11;
const uint8_t SET_CONTROLLER_PROPERTIES = 0x12;
const uint8_t GET_CONTROLLER_PROPERTIES = 0x13;
const uint8_t INIT_MOTORS = 0x14;

const uint8_t GET_PID_CONSTANTS = 0x20;
const uint8_t SET_PID_CONSTANTS = 0x21;

const uint8_t SET_MOTOR_PWMS = 0x40;
const uint8_t GET_MOTOR_PWMS = 0x41;
const uint8_t SET_MOTOR_SPEED_SETPOINTS = 0x42;
const uint8_t GET_MOTOR_SPEED = 0x43;
const uint8_t SET_MOTOR_ANGLE_SETPOINTS = 0x44;
const uint8_t GET_MOTOR_ANGLE = 0x45;
const uint8_t SET_MOTOR_CONTROL_MODES = 0x46;
const uint8_t GET_MOTOR_CONTROL_MODES = 0x47;

const uint8_t SET_ODO_BROADCAST_STATUS = 0X60;
const uint8_t SEND_ODO_SPEEDS = 0x65;
const uint8_t SEND_ODO_ANGLES = 0x66;
const uint8_t SEND_ODO_PWMS = 0x67;

#endif // COMMANDS_H
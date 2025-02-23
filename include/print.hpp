#pragma once

#include <iostream>
#include "stdint.h"
#include "Structs.hpp"

using namespace std;


void printOdoBroadcastStatus(const OdoBroadcastStatus &status);
void printPIDConstants(const PIDConstants &pid);
void printPWMLimits(const PWMLimits &limits);
void printMotorConnections(const MotorConnections &conn);
void printOdometryData(const OdometryData &data);
void printSetpoint(const Setpoint &setpoint);
void printUpdateFrequenciesWheel(const UpdateFrequenciesWheel &freqs);
void printControlMode(const ControlMode &mode);
void printMotorData(const MotorData &motor);
void printUpdateFrequencies(const UpdateFrequencies &freqs);
void printControllerProperties(const ControllerProperties &props);
void printControllerData(const ControllerData &controller);


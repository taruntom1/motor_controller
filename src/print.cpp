#include "print.hpp"

void printOdoBroadcastStatus(const OdoBroadcastStatus &status)
{
    cout << "OdoBroadcastStatus: angleBroadcast=" << status.angleBroadcast
         << ", speedBroadcast=" << status.speedBroadcast
         << ", pwmBroadcast=" << status.pwmBroadcast << endl;
}

void printPIDConstants(const PIDConstants &pid)
{
    cout << "PIDConstants: P=" << pid.p << ", I=" << pid.i << ", D=" << pid.d << endl;
}

void printPWMLimits(const PWMLimits &limits)
{
    cout << "PWMLimits: minPWM=" << (int)limits.minPWM << ", maxPWM=" << (int)limits.maxPWM << endl;
}

void printMotorConnections(const MotorConnections &conn)
{
    cout << "MotorConnections: dirPin=" << (int)conn.dirPin
         << ", pwmPin=" << (int)conn.pwmPin
         << ", encPinA=" << (int)conn.encPinA
         << ", encPinB=" << (int)conn.encPinB << endl;
}

void printOdometryData(const OdometryData &data)
{
    cout << "OdometryData: angle=" << data.angle << ", rpm=" << data.rpm << endl;
}

void printSetpoint(const Setpoint &setpoint)
{
    cout << "Setpoint: angle=" << setpoint.angle << ", rpm=" << setpoint.rpm << endl;
}

void printUpdateFrequenciesWheel(const UpdateFrequenciesWheel &freqs)
{
    cout << "UpdateFrequenciesWheel: pwm=" << freqs.pwm
         << ", anglePID=" << freqs.anglePID
         << ", speedPID=" << freqs.speedPID << endl;
}

void printControlMode(const ControlMode &mode)
{
    cout << "ControlMode: ";
    switch (mode)
    {
    case POSITION_CONTROL:
        cout << "POSITION_CONTROL";
        break;
    case SPEED_CONTROL:
        cout << "SPEED_CONTROL";
        break;
    case PWM_DIRECT_CONTROL:
        cout << "PWM_DIRECT_CONTROL";
        break;
    case OFF:
        cout << "OFF";
        break;
    }
    cout << endl;
}

void printMotorData(const MotorData &motor)
{
    cout << "MotorData: motorID=" << (int)motor.motorID << endl;
    printControlMode(motor.controlMode);
    printPIDConstants(motor.anglePIDConstants);
    printPIDConstants(motor.speedPIDConstants);
    printPWMLimits(motor.pwmLimits);
    printMotorConnections(motor.motorConnections);
    printOdometryData(motor.odometryData);
    printSetpoint(motor.setpoint);
    printOdoBroadcastStatus(motor.odoBroadcastStatus);
    printUpdateFrequenciesWheel(motor.updateFrequenciesWheel);
    cout << "PWM Value: " << motor.pwmValue << endl;
}

void printUpdateFrequencies(const UpdateFrequencies &freqs)
{
    cout << "UpdateFrequencies: interfaceRun=" << freqs.interfaceRun << endl;
}

void printControllerProperties(const ControllerProperties &props)
{
    cout << "ControllerProperties: run=" << props.run
         << ", numMotors=" << (int)props.numMotors
         << ", odoBroadcastFrequency=" << props.odoBroadcastFrequency << endl;
    printOdoBroadcastStatus(props.odoBroadcastStatus);
    printUpdateFrequencies(props.updateFrequencies);
}

void printControllerData(const ControllerData &controller)
{
    cout << "ControllerData:" << endl;
    for (uint8_t i = 0; i < controller.controllerProperties.numMotors; ++i)
    {
        cout << "\nMotor " << (int)i << ":" << endl;
        printMotorData(controller.motorData[i]);
    }
    printControllerProperties(controller.controllerProperties);
}

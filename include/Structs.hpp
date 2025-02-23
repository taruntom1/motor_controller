#pragma once
// In MyStructs.h
/**
 * @file MyStructs.h
 * @brief Header file for the structs used in the communication interface
 *  class, responsible for managing motor control and communication.
 */
#ifndef MYSTRUCTS_H
#define MYSTRUCTS_H

#include <stdint.h>


 /**
  * @defgroup Structs
  * @brief Structs for storing different properties and setpoints of the controllers
  * @{
  */

  /**
   * @struct OdoBroadcastStatus
   * @brief Represents the status of odometry data broadcasting.
   */
struct OdoBroadcastStatus
{
    bool angleBroadcast = false; ///< Indicates if angle data is being broadcast.
    bool speedBroadcast = false; ///< Indicates if speed data is being broadcast.
    bool pwmBroadcast = false;   ///< Indicates if PWM data is being broadcast.
};

/**
 * @struct PIDConstants
 * @brief Contains PID constants for motor control.
 */
struct PIDConstants
{
    float p = 0; ///< Proportional constant.
    float i = 0; ///< Integral constant.
    float d = 0; ///< Derivative constant.
};

/**
 * @struct PWMLimits
 * @brief Defines the PWM limits for motor control.
 */
struct PWMLimits
{
    int8_t minPWM = 0; ///< Minimum PWM value.
    int8_t maxPWM = 0; ///< Maximum PWM value.
};

/**
 * @struct MotorConnections
 * @brief Represents the hardware connections for a motor.
 */
struct MotorConnections
{
    uint8_t dirPin;  ///< Direction pin.
    uint8_t pwmPin;  ///< PWM pin.
    uint8_t encPinA; ///< Encoder pin A.
    uint8_t encPinB; ///< Encoder pin B.
};

/**
 * @struct OdometryData
 * @brief Stores odometry information for a motor.
 */
struct OdometryData
{
    int angle = 0; ///< Current angle in degrees.
    float rpm = 0; ///< Current speed in RPM.
};

/**
 * @struct Setpoint
 * @brief Represents desired values for motor control.
 */
struct Setpoint
{
    int angle = 0; ///< Desired angle in degrees.
    float rpm = 0; ///< Desired speed in RPM.
};

/**
 * @struct UpdateFrequencyWheel
 * @brief different loop update frequency for each wheel
 */
struct UpdateFrequenciesWheel
{
    uint16_t pwm = 50;      ///< PWM update frequency in Hz when on Direct PWM control.
    uint16_t anglePID = 50; ///< Angle PID update frequency in Hz when on Position control mode.
    uint16_t speedPID = 50; ///< Speed PID update frequency in Hz when on speed control mode.
};

/**
 * @brief ControlMode enum
 * @brief Enumerates different control modes for motor operation.
 */
enum ControlMode
{
    POSITION_CONTROL,   ///< Position control mode.
    SPEED_CONTROL,      ///< Speed control mode.
    PWM_DIRECT_CONTROL, ///< Direct PWM control mode.
    OFF                 ///< Off

};

/**
 * @struct MotorData
 * @brief Encapsulates all data related to a motor.
 */
struct MotorData
{
    uint8_t motorID = 0;                           ///< Unique identifier for the motor.
    ControlMode controlMode = PWM_DIRECT_CONTROL;  ///< Control mode for the motor.
    PIDConstants anglePIDConstants;                ///< PID constants for angle control.
    PIDConstants speedPIDConstants;                ///< PID constants for speed control.
    PWMLimits pwmLimits;                           ///< PWM limits.
    MotorConnections motorConnections;             ///< Hardware connections.
    OdometryData odometryData;                     ///< Odometry data.
    Setpoint setpoint;                             ///< Desired setpoints.
    OdoBroadcastStatus odoBroadcastStatus;         ///< Status of odometry broadcasting.
    UpdateFrequenciesWheel updateFrequenciesWheel; ///< Different update frequencies for wheel control.
    int16_t pwmValue = 0;                         ///< Current PWM value.
};

/**
 * @struct LoopingFrequencys
 * @brief Defines different update frequencies for various components.
 */

struct UpdateFrequencies
{
    uint16_t interfaceRun = 50;
};

/**
 * @struct ControllerProperties
 * @brief Defines properties of the motor controller.
 */
struct ControllerProperties
{
    bool run = false;                      ///< Indicates if the controller is active.
    uint8_t numMotors = 2;                 ///< Number of motors controlled.
    OdoBroadcastStatus odoBroadcastStatus; ///< Status of odometry broadcasting.
    uint16_t odoBroadcastFrequency = 30;   ///< Frequency of odometry broadcasting.
    UpdateFrequencies updateFrequencies;   ///< Different update frequencies for various components.
};

/**
 * @struct ControllerData
 * @brief Contains all data for the motor controller.
 */
struct ControllerData
{
    MotorData* motorData = nullptr;            ///< Pointer to an array of MotorData structures.
    ControllerProperties controllerProperties; ///< Properties of the controller.
};




#endif // MYSTRUCTS_H

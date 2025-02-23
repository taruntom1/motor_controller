#pragma once
#include "UARTProtocol.hpp"
#include "Structs.hpp"
#include "Commands.hpp"

/**
 * @class ControlInterface
 * @brief Interface for controlling a motor system using UART communication.
 */
class ControlInterface
{
private:
    ProtocolConfig protocol_config; ///< Configuration parameters for the communication protocol.
    UARTProtocol protocol;          ///< UART protocol instance for communication.
    ControllerData &controllerData; ///< Reference to controller data structure.

public:
    /**
     * @brief Constructs a ControlInterface object.
     * @param protocol_config Configuration settings for the protocol.
     * @param controllerData Reference to the controller data structure.
     */
    ControlInterface(ProtocolConfig protocol_config, ControllerData &controllerData);

    /**
     * @brief Runs the control interface loop.
     */

    void Run();

    /**
     * @brief Sends a ping command to check communication status.
     * @return True if the ping response is received, false otherwise.
     */
    bool Ping();

    /**
     * @brief Sends motor-specific data to the controller.
     * @param motor_id The ID of the motor.
     * @return True if the data was successfully sent, false otherwise.
     */
    bool SetMotorData(uint8_t motor_id);

    /**
     * @brief Sends controller properties (e.g., settings, configurations) to the motor controller.
     * @return True if successful, false otherwise.
     */
    bool SetControllerProperties();

    /**
     * @brief Sets the control mode of the motor (e.g., position, velocity, direct pwm).
     * @return True if the control mode was set successfully, false otherwise.
     */
    bool SetMotorControlMode();

    /**
     * @brief Sends PWM values to control motor speed
     */
    void SetMotorPWMs();

    /**
     * @brief Enables or disables odometry data broadcast for a specific motor.
     * @param motor_id The ID of the motor.
     * @return True if successful, false otherwise.
     */
    bool SetOdoBroadcastStatus(uint8_t motor_id);

    /**
     * @brief Receives odometry speed data from the motor controller.
     * @return True if the data was successfully received, false otherwise.
     */
    bool RecieveOdoSpeeds();

    /**
     * @brief Receives odometry angle data from the motor controller.
     * @return True if the data was successfully received, false otherwise.
     */
    bool RecieveOdoAngles();
};
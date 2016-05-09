// Written by Andrew Donelick
// adonelick@hmc.edu

// This class contains code for controlling the attitude of a 
// high altitude balloon payload. At the moment, you can only control
// the yaw of the payload, not pitch and roll. 


#ifndef ATTITUDE_CONTROLLER_H
#define ATTITUDE_CONTROLLER_H 1

#include <inttypes.h>

#if ARDUINO >= 100
#include "Arduino.h"       // for delayMicroseconds, digitalPinToBitMask, etc
#else
#include "WProgram.h"      // for delayMicroseconds
#include "pins_arduino.h"  // for digitalPinToBitMask, etc
#endif

#define PITCH 0
#define ROLL 1
#define YAW 2

#define PLUS 0
#define MINUS 1


class AttitudeController
{

private:

    // Pins which control the attitude actuators
    uint8_t pins_[3][2];

    // Desired state to attain using the controller
    int32_t desiredState_[3];

    // Current attitude state of the payload
    int32_t actualState_[3];
    uint32_t time_[2];

    // Gains for the PID controller for all three axes
    int32_t p_gain_[3];
    int32_t i_gain_[3];
    int32_t d_gain_[3];

    // Error terms for the controller
    int32_t proportionalError_[3];
    int32_t integralError_[3];
    int32_t derivativeError_[3];

    // Actuation thresholds for the controller
    int32_t thresholds_[3];

    // Time to wait before changing the command
    uint32_t waitTime_;
    uint32_t lastUpdateTime_;

public:

    AttitudeController(uint32_t waitTime);

    // Prepares the controller for operation
    void begin();

    // Sets the pin numbers which control the actuators
    // Pin order: {PLUS, MINUS}
    void setActuatorPins(uint8_t axis, uint8_t plus, uint8_t minus);

    // Sets the error threshold at which activation is triggered
    void setActuationThreshold(int32_t threshold, uint8_t axis);

    // Sets the controller gains for the specified axis 
    void setGains(uint8_t axis, int32_t p, int32_t i, int32_t d);

    // Updates the controller's current attitude state to use 
    // in correcting the payload's attitude to match the desired state
    // Units:
    //      Pitch, roll, yaw - hundredths of degrees
    //      time: milliseconds
    void updateState(int32_t pitch, int32_t roll, int32_t yaw, uint32_t time);

    // Update the state of the actuators based on the current readings from
    // our attitude measuring sensors
    void updateActuators();

    // Sets the desired pitch, roll, and yaw values to target
    // Units:
    //      Pitch, roll, yaw - hundredths of degrees
    void setDesiredState(int32_t pitch, int32_t roll, int32_t yaw);


private:

    // Update the error terms
    void updateErrors();

};


#endif
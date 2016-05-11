// Written by Andrew Donelick
// adonelick@hmc.edu

#include "AttitudeController.h"

AttitudeController::AttitudeController(uint32_t waitTime)
    : waitTime_(waitTime),
      lastUpdateTime_(0),
      enabled_(false)
{
    // Nothing to do here...
}


void AttitudeController::begin()
{
    // Sets all the actuator pins as outputs, turns all actuators off
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 2; ++j) {
            pinMode(pins_[i][j], OUTPUT);
            digitalWrite(pins_[i][j], LOW);
        }
    }
}


void AttitudeController::enable()
{
    enabled_ = true;
}


void AttitudeController::disable()
{
    enabled_ = false;

    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 2; ++j) {
            pinMode(pins_[i][j], OUTPUT);
            digitalWrite(pins_[i][j], LOW);
        }
    }

    // Reset the correction errors to zero
    proportionalError_[PITCH] = 0;
    proportionalError_[ROLL] = 0;
    proportionalError_[YAW] = 0;

    integralError_[PITCH] = 0;
    integralError_[ROLL] = 0;
    integralError_[YAW] = 0;

    derivativeError_[PITCH] = 0;
    derivativeError_[ROLL] = 0;
    derivativeError_[YAW] = 0;
}


void AttitudeController::setActuatorPins(uint8_t axis, uint8_t plus, uint8_t minus)
{
    pins_[axis][PLUS] = plus;
    pins_[axis][MINUS] = minus;
}


void AttitudeController::setActuationThreshold(uint8_t axis, int32_t threshold)
{
    thresholds_[axis] = threshold;
}


void AttitudeController::setGains(uint8_t axis, int32_t p, int32_t i, int32_t d)
{
    p_gain_[axis] = p;
    i_gain_[axis] = i;
    d_gain_[axis] = d;
}


void AttitudeController::updateState(int32_t pitch, int32_t roll, int32_t yaw, uint32_t newTime)
{
    actualState_[PITCH] = normalizeAngle(pitch);
    actualState_[ROLL] = normalizeAngle(roll);
    actualState_[YAW] = normalizeAngle(yaw);

    uint32_t oldTime = time_[1];
    time_[1] = newTime;
    time_[0] = oldTime;
}


void AttitudeController::setDesiredState(int32_t pitch, int32_t roll, int32_t yaw)
{
    desiredState_[PITCH] = normalizeAngle(pitch);
    desiredState_[ROLL] = normalizeAngle(roll);
    desiredState_[YAW] = normalizeAngle(yaw);
}


void AttitudeController::updateActuators()
{
    // Based on the current actuation signal, command the attitude
    // actuators to do what they need to do

    if (!enabled_) {
        return;
    }

    for (uint8_t axis = 0; axis < 3; ++axis)
    {
        int32_t p = p_gain_[axis];
        int32_t i = i_gain_[axis];
        int32_t d = d_gain_[axis];

        // Calculate the actuation from the compensator
        int32_t actuation = p*proportionalError_[axis];
        actuation += i*integralError_[axis];
        actuation += d*derivativeError_[axis];

        // If we have passed enough time from the last actuation,
        // go ahead and update the actuators
        if (millis() - lastUpdateTime_ > waitTime_) {
            
            if (abs(actuation) >= thresholds_[axis]) {

                // If we are above the threshold, turn on 
                // the correct pin, and the other turn off
                if (actuation > 0) {
                    digitalWrite(pins_[axis][PLUS], HIGH);
                    digitalWrite(pins_[axis][MINUS], LOW);   
                } else {
                    digitalWrite(pins_[axis][PLUS], LOW);
                    digitalWrite(pins_[axis][MINUS], HIGH); 
                }
                
            } else {
                // Otherwise, turn both pins for the axis off
                digitalWrite(pins_[axis][PLUS], LOW);
                digitalWrite(pins_[axis][MINUS], LOW); 
            }
        }
    }
}


void AttitudeController::updateErrors()
{
    if (!enabled_) {
        return;
    }

    int32_t pitchError = desiredState_[PITCH] - actualState_[PITCH];
    int32_t rollError = desiredState_[ROLL] - actualState_[ROLL];
    int32_t yawError = desiredState_[YAW] - actualState_[YAW];

    uint32_t dt = time_[1] - time_[0];

    proportionalError_[PITCH] = pitchError;
    proportionalError_[ROLL] = rollError;
    proportionalError_[YAW] = yawError;

    integralError_[PITCH] += (pitchError*dt) / 1000;
    integralError_[ROLL] += (rollError*dt) / 1000;
    integralError_[YAW] += (yawError*dt) / 1000;

    derivativeError_[PITCH] = 0;
    derivativeError_[ROLL] = 0;
    derivativeError_[YAW] = 0;
}


int32_t AttitudeController::normalizeAngle(int32_t angle)
{

    angle = angle % 36000;

    if (angle > 18000) {
        return angle - 36000;
    } else if (angle < -18000) {
        return angle + 36000;
    } else {
        return angle;
    }
}
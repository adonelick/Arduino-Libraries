// Written by Andrew Donelick
// adonelick@hmc.edu

#include "AttitudeController.h"

AttitudeController::AttitudeController(uint32_t waitTime)
    : waitTime_(waitTime),
      lastUpdateTime_(0),
      enabled_(false),
      numPoints_(0)
{
    // Nothing to do here...
}


void AttitudeController::begin()
{
    // Sets all the actuator pins as outputs, turns all actuators off
    for (int i = 0; i < 3; ++i) {
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


bool AttitudeController::enabled()
{
    return enabled_;
}

void AttitudeController::disable()
{

    if (!enabled_) {
        return;
    }

    enabled_ = false;

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 2; ++j) {
            pinMode(pins_[i][j], OUTPUT);
            digitalWrite(pins_[i][j], LOW);
        }
    }

    // Reset some of the correction errors to zero
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


int32_t AttitudeController::getActuation(uint8_t axis)
{
    int32_t p = p_gain_[axis];
    int32_t i = i_gain_[axis];
    int32_t d = d_gain_[axis];

    // Calculate the actuation from the compensator
    int32_t actuation = proportionalError_[axis][0]/p;
    actuation += integralError_[axis]/i;
    actuation += derivativeError_[axis]/d;

    if (actuation > MAX_ACTUATION) {
        actuation = MAX_ACTUATION;
    } else if (actuation < -MAX_ACTUATION) {
        actuation = -MAX_ACTUATION;
    }

    return actuation;
}


void AttitudeController::setGains(uint8_t axis, int32_t p, int32_t i, int32_t d)
{
    p_gain_[axis] = p;
    i_gain_[axis] = i;
    d_gain_[axis] = d;
}


void AttitudeController::updateState(int32_t pitch, int32_t roll, int32_t yaw, uint32_t newTime)
{
    // Shift all the previous saved time entries back
    for (uint16_t index = POINTS_TO_STORE - 1; index > 0; --index) {
        time_[index] = time_[index - 1];
    }

    // Save the most recent attitue and time reading
    actualState_[PITCH] = normalizeAngle(pitch);
    actualState_[ROLL] = normalizeAngle(roll);
    actualState_[YAW] = normalizeAngle(yaw);
    time_[0] = (int32_t) newTime;

    // Increment the counter of the number of points stored
    if (numPoints_ < POINTS_TO_STORE) {
        ++numPoints_;
    }
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

    for (uint8_t axis = 0; axis < 3; ++axis) {
        int32_t actuation = getActuation(axis);

        // If we have passed enough time from the last actuation,
        // go ahead and update the actuators
        if (millis() - lastUpdateTime_ > waitTime_) {
            
            if (abs(actuation) >= thresholds_[axis]) {

                // If we are above the threshold, turn on 
                // the correct pin, and the other turn off
                if (actuation > 0) {
                    analogWrite(pins_[axis][PLUS], abs(actuation));
                    analogWrite(pins_[axis][MINUS], 0);   
                } else {
                    analogWrite(pins_[axis][PLUS], 0);
                    analogWrite(pins_[axis][MINUS], abs(actuation));
                }
                
            } else {
                // Otherwise, turn both pins for the axis off
                analogWrite(pins_[axis][PLUS], 0);
                analogWrite(pins_[axis][MINUS], 0); 
            }
        }
    }
}


void AttitudeController::updateErrors()
{
    // Set the error to be the difference between the most recent 
    // state reading and the desired state
    int32_t pitchError = desiredState_[PITCH] - actualState_[PITCH];
    int32_t rollError = desiredState_[ROLL] - actualState_[ROLL];
    int32_t yawError = desiredState_[YAW] - actualState_[YAW];
    int32_t dt = time_[0] - time_[1];

    // Shift all the previous saved error entries back
    for (uint16_t index = POINTS_TO_STORE - 1; index > 0; --index) {
        proportionalError_[PITCH][index] = proportionalError_[PITCH][index - 1];
        proportionalError_[ROLL][index] = proportionalError_[ROLL][index - 1];
        proportionalError_[YAW][index] = proportionalError_[YAW][index - 1];
    }

    proportionalError_[PITCH][0] = pitchError;
    proportionalError_[ROLL][0] = rollError;
    proportionalError_[YAW][0] = yawError;

    // Prevent integrator wind-up by returning if we are not actually 
    // controlling the payload's attitude
    if (!enabled_) {
        return;
    }

    // Calculate the integral of the error
    integralError_[PITCH] += (pitchError*dt) / 1000;
    integralError_[ROLL] += (rollError*dt) / 1000;
    integralError_[YAW] += (yawError*dt) / 1000;

    // Calculate the derivative of the error (with noise reduction)
    derivativeError_[PITCH] = calculateSlope(time_, proportionalError_[PITCH]);
    derivativeError_[ROLL] = calculateSlope(time_, proportionalError_[ROLL]);
    derivativeError_[YAW] = calculateSlope(time_, proportionalError_[YAW]);
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


int32_t AttitudeController::calculateSlope(int32_t x[], int32_t y[])
{
    // Use the following page to calculate the slope of a line
    // fit to the points contained in the arrays x, y
    // https://en.wikipedia.org/wiki/Simple_linear_regression#Fitting_the_regression_line

    // Calculate the mean of the x and y points
    int32_t meanX = mean(x);
    int32_t meanY = mean(y);

    // Calculate the numerator
    int32_t numerator = 0;
    for (uint16_t i = 0; i < numPoints_; ++i) {
        numerator += (x[i] - meanX)*(y[i] - meanY);
    }
    numerator *= 1000;

    // Calculate the denominator
    int32_t denominator = 0;
    for (uint16_t i = 0; i < numPoints_; ++i) {
        denominator += (x[i] - meanX)*(x[i] - meanX);
    }

    return numerator / denominator;
}


int32_t AttitudeController::mean(int32_t array[])
{
    // Calculate the integer mean of an array
    int32_t sum = 0;
    for (uint16_t i = 0; i < numPoints_; ++i) {
        sum += array[i];
    }

    return sum / numPoints_;
}


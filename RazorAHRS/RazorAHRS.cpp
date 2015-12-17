// Written by Andrew Donelick
// 13 December 2015
// adonelick@hmc.edu

#include "RazorAHRS.h"

RazorAHRS::RazorAHRS(HardwareSerial& razorSerial)
    : razorSerial_(razorSerial)
{
    // Nothing else to do here...
}

void RazorAHRS::begin()
{
    razorSerial_.begin(57600);
    razorSerial_.write("#ob");  // Turn on binary output
    razorSerial_.write("#o1");  // Turn on continuous streaming output
    razorSerial_.write("#oe0"); // Disable error message output
}


bool RazorAHRS::available()
{
    // We require at least 32 bytes of data waiting at the serial port,
    // so that no matter where in the message we start decoding,
    // we can recover yaw, pitch, and roll.
    return razorSerial_.available() > 32;
}


bool RazorAHRS::decodeMessage()
{
    // If we don't have enough data, don't even bother
    if (not available()) {
        return false;
    }

    for (uint16_t i = 0; i < 32; ++i)
    {
        byteArray_[i] = razorSerial_.read();
    }

    // Find the start of the message, pull out the bytes for
    // the yaw, pitch and roll components of the message
    for (uint16_t i = 0; i < 32; ++i)
    {
      if (startYPR(i))
      {
        yaw_.asBytes[0] = byteArray_[i+4];
        yaw_.asBytes[1] = byteArray_[i+5];
        yaw_.asBytes[2] = byteArray_[i+6];
        yaw_.asBytes[3] = byteArray_[i+7];
        
        pitch_.asBytes[0] = byteArray_[i+8];
        pitch_.asBytes[1] = byteArray_[i+9];
        pitch_.asBytes[2] = byteArray_[i+10];
        pitch_.asBytes[3] = byteArray_[i+11];
        
        roll_.asBytes[0] = byteArray_[i+12];
        roll_.asBytes[1] = byteArray_[i+13];
        roll_.asBytes[2] = byteArray_[i+14];
        roll_.asBytes[3] = byteArray_[i+15];
        break;
      }
    }

    return true;
}


bool RazorAHRS::startYPR(uint16_t index)
{
    bool checkOne = byteArray_[index] == 'Y';
    bool checkTwo = byteArray_[index+1] == 'P';
    bool checkThree = byteArray_[index+2] == 'R';
    bool checkFour = byteArray_[index+3] == ':';

    return checkOne && checkTwo && checkThree && checkFour;
}


float RazorAHRS::getYaw()
{
    return yaw_.asFloat;
}


float RazorAHRS::getPitch()
{
    return pitch_.asFloat;
}


float RazorAHRS::getRoll()
{
    return roll_.asFloat;
}

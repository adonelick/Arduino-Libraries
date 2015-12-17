// Written by Andrew Donelick
// 13 December 2015
// adonelick@hmc.edu

#ifndef RAZOR_AHRS_H
#define RAZOR_AHRS_H 1

#if ARDUINO >= 100
#include "Arduino.h"       // for delayMicroseconds, digitalPinToBitMask, etc
#else
#include "WProgram.h"      // for delayMicroseconds
#include "pins_arduino.h"  // for digitalPinToBitMask, etc
#endif


class RazorAHRS 
{
    private:

        // Serial port used to communicate with the Razor
        HardwareSerial& razorSerial_;
        
        // Byte array used to store values read from the Razor
        char byteArray_[32];

        // Union which converts the yaw bytes into a float 
        union {
          byte asBytes[4];
          float asFloat;
        } yaw_;

        // Union which converts the pitch bytes into a float 
        union {
          byte asBytes[4];
          float asFloat;
        } pitch_;

        // Union which converts the roll bytes into a float 
        union {
          byte asBytes[4];
          float asFloat;
        } roll_;


    public:

        // Constructor for the class
        RazorAHRS(HardwareSerial& razorSerial);

        // Initializes the Razor, prepares it for communication
        void begin();

        // Checks if there is a yaw-pitch-roll message waiting to be read
        bool available();

        // Reads the bytes from the serial port, decodes them into yaw,
        // pitch, and roll values. Returns whether or not decoding succeeded
        bool decodeMessage();

        // Returns the yaw last decoded from the Razor
        float getYaw();

        // Returns the pitch last decoded from the Razor
        float getPitch();

        // Returns the roll last decoded from the Razor
        float getRoll();

    private:

        // Determine if the yaw-pitch-roll message starts from
        // the given index
        bool startYPR(uint16_t index);

};



#endif // RAZOR_AHRS_H
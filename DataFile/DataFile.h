// Written by Andrew Donelick
// 13 December 2015
// adonelick@hmc.edu

/*
 * This class provides a wrapper for the SD library for use in logging
 * data collected by the balloon. Functionality includes constructing 
 * and writing a file header, automatically generating file names for 
 * the files, and an easy interface for logging individual entries.
 */

#ifndef DATAFILE
#define DATAFILE_H 1

#include "SD.h"

#if ARDUINO >= 100
#include "Arduino.h"       // for delayMicroseconds, digitalPinToBitMask, etc
#else
#include "WProgram.h"      // for delayMicroseconds
#include "pins_arduino.h"  // for digitalPinToBitMask, etc
#endif

// Reserve pins 10 and 53 for use with the SD card

#define NUM_ENTRIES 20

class DataFile
{

    private:
        File dataFile_;
        char* filename_;
        char* entries_[NUM_ENTRIES];
        int numEntries_;
        int currentEntry_;

    public:

        // Constructor which automatically builds a filename
        DataFile();

        // Creates the filename, sets up the SD card communication
        void begin();

        // Opens the data file and prepares it for writing
        void open();

        // Closes the data file
        void close();

        // Add an entry name to the data file. Data entries should be made
        // in the order that this function was callled.
        void addEntry(char* entryName); 

        // Writes the descriptive header for the measurements in a 
        // specific data file
        void writeFileHeader();

        // Writes a new entry to the datafile
        void writeEntry(int value);
        void writeEntry(unsigned int value);
        void writeEntry(unsigned long value);
        void writeEntry(float value);
        void writeEntry(bool value);
        void writeEntry(char* value);

        // Checks the status of the data file (whether or not is successfully
        // opened and it working)
        bool checkStatus();

    private:

        // Writes a newline to the file
        void writeNewLine();

        // Writes the time the entry was made (in milliseconds since the 
        // Arduino started running)
        void writeEntryTime();

        // Writes either a comma or newline after an entry, depending
        // on whether the entry was the last in the line, or not
        void writeEntryEnd();
};



#endif

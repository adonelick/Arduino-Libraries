// Written by Andrew Donelick
// 13 December 2015
// adonelick@hmc.edu


#include "DataFile.h"


DataFile::DataFile()
    : numEntries_(0),
      currentEntry_(0)
{
    filename_ = "DATA000.CSV";
}

void DataFile::begin()
{
    // Start the SD card
    // This pin needs to be set to output for writing to the
    // SD card, even if we do not use it
    pinMode(10, OUTPUT);
    pinMode(53, OUTPUT);
    SD.begin(53);

    int i = 1;
    while (SD.exists(filename_) && (i < 999))
    {
        filename_[4] = i/100 + '0';
        filename_[5] = (i%100)/10 + '0';
        filename_[6] = (i%100)%10 + '0';
        ++i;
    }
}

void DataFile::open()
{
    dataFile_ = SD.open(filename_, FILE_WRITE);
}

void DataFile::close()
{
    dataFile_.close();
}

void DataFile::addEntry(char* entryName)
{
    entries_[numEntries_] = entryName;
    ++numEntries_;
}

void DataFile::writeFileHeader()
{
    open();
    if (dataFile_)
    {
        for (int i = 0; i < numEntries_; ++i)
        {
            dataFile_.print("Time,");
            dataFile_.print(entries_[i]);

            if (i != numEntries_ - 1)
            {
                dataFile_.print(',');
            }
        }

        writeNewLine();
        close();
    }
}

// This function assumes that the file is open
void DataFile::writeEntry(int value)
{
    if (!dataFile_)
        return;
    writeEntryTime();
    dataFile_.print(value);
    writeEntryEnd();
}


void DataFile::writeEntry(unsigned int value)
{
    if (!dataFile_)
        return;
    writeEntryTime();
    dataFile_.print(value);
    writeEntryEnd();
}

void DataFile::writeEntry(unsigned long value)
{
    if (!dataFile_)
        return;
    writeEntryTime();
    dataFile_.print(value);
    writeEntryEnd();
}

void DataFile::writeEntry(float value)
{
    if (!dataFile_)
        return;
    writeEntryTime();
    dataFile_.print(value);
    writeEntryEnd();
}


void DataFile::writeEntry(bool value)
{
    if (!dataFile_)
        return;

    writeEntryTime();
    if (value) {
        dataFile_.print("True");
    } else {
        dataFile_.print("False");
    }
    writeEntryEnd();
}

void DataFile::writeEntry(char* value)
{
    if (!dataFile_)
        return;
    writeEntryTime();
    dataFile_.print(value);
    writeEntryEnd();
}


void DataFile::writeEntryTime()
{
    if (!dataFile_)
        return;

    dataFile_.print(millis());
    dataFile_.print(',');
}


void DataFile::writeEntryEnd()
{
    
    if (!dataFile_)
        return;

    ++currentEntry_;

    if (currentEntry_ != numEntries_) {
        dataFile_.print(',');
    } else {
        currentEntry_ = 0;
        writeNewLine();
    }
}


// This function assumes that the ile is open
void DataFile::writeNewLine()
{
    if (dataFile_)
    {
        dataFile_.println();
    }
}

bool DataFile::checkStatus()
{
    if (dataFile_) {
        return true;
    } else {
        return false;
    }
}
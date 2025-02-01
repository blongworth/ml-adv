/*!
 * @file adv.h
 *
 * This is a library for working with a NORTEK ADV
 * Reads serial to a data structure without blocking
 *
 * Written by Brett Longworth
 *
 * BSD license, all text above must be included in any redistribution
 */
#ifndef ADV_h
#define ADV_h

#include <Arduino.h>

// #define ADV_SERIAL Serial3
// no longer needed. Use appropriate Serial object in constructor
const byte VVDChar = 16; //VVD packet designator
const byte VVDLength = 24; //length of VVD packets
const byte VSDLength = 28; //length of VSD packets
const byte numChars = 28; //length of ADV packets
const byte startMarker = 165; //start byte of ADV packets

struct TimeData
{
    byte Year;
    byte Month;
    byte Day;
    byte Hour;
    byte Minute;
    byte Second;
};

struct VectorSystemData
{
    byte Sync;
    byte Id;
    unsigned short Size;
    TimeData Time;
    unsigned short Battery;
    unsigned short Soundspeed;
    short Heading;
    short Pitch;
    short Roll;
    short Temperature;
    char Error;
    char Status;
    unsigned short AnaIn;
    short Checksum;
};

struct VectorVelocityData
{
    byte Sync;
    byte Id;
    byte Count;
    int Pressure;
    unsigned short AnaIn1;
    unsigned short AnaIn2;
    byte Amplitude[3];
    byte Correlation[3];
    short Velocity[3];
    short Checksum;
};

class ADV
{
private:
    byte ADVpacket[numChars];
    boolean newData;
    Stream &serial;
    
    void read_serial();
    int BCD_Convert(int bit8);
    int s16bit(int bit8a, int bit8b);
    void parseVSD(byte buf[VSDLength], VectorSystemData vsd);
    void parseVVD(byte buf[VVDLength], VectorVelocityData vvd); //see p37 of Integration Manual for vvd structure

public:
    ADV(Stream &serial);
    void begin();
    void read();
    boolean VVDReady;
    boolean VSDReady;
    int getVSD();
    int getVVD();
    int getVSDPacket();
    int getVVDPacket();
};

#endif
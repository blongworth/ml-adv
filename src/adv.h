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

class ADV
{
private:
    byte ADVpacket[numChars];
    boolean newData;
    Stream &serial;
    
    void read_serial();
    int BCD_Convert(int bit8);
    int s16bit(int bit8a, int bit8b);
    void parseVSD(byte buf[VSDLength], int VSD[]);
    void parseVVD(byte buf[VVDLength], int VVD[]); //see p37 of Integration Manual for vvd structure

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
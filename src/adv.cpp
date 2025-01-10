/*!
 * @file adv.cpp
 *
 * @mainpage Nortek ADV Library
 *
 * @section intro_sec Introduction
 *
 * This is a library for working with a NORTEK ADV
 * Reads serial to a data structure without blocking
 *
 * @section author Author
 *
 * Written by Brett Longworth
 *
 * @section license License
 *
 * BSD license, all text above must be included in any redistribution
 */

#include <Arduino.h>
#include "adv.h"

ADV::ADV(Stream &serial) : serial(serial)
{
    newData = false;
    VVDReady = false;
    VSDReady = false;
}

void ADV::begin() {
  serial.write("@@@@@@");
  delay(200);
  serial.write("K1W%!Q");
  delay(200);
  serial.write("SR");
}

void ADV::read()
{
  read_serial();
  if (newData)
  {
    if (ADVpacket[1] == VVDChar)
    {
      VVDReady = 1;
    }
    else
    {
      VSDReady = 1;
    }
    //newData = false;
  }
}

void ADV::read_serial() {
  static byte ndx = 0;
  static boolean recvInProgress = false;
  static byte packetLength;
  byte rc;
  while (serial.available() > 0 && newData == false) {
    rc = serial.read();
    if (recvInProgress == true) {
      if (ndx == 1) {
        if (rc == VVDChar) {
          packetLength = VVDLength;
        } else {
          packetLength = VSDLength;
        }
        ADVpacket[ndx] = rc;
        ndx++;
      } else if (ndx == packetLength - 1) { // whole packet received
        ADVpacket[ndx] = rc;
        ndx++;
        ADVpacket[ndx] = '\0';
        ndx = 0;
        newData = true;
        recvInProgress = false;
      } else {
        ADVpacket[ndx] = rc;
        ndx++;
      }
    } else if (rc == startMarker) {
      ADVpacket[ndx] = rc;
      ndx++;
      recvInProgress = true;
    }
  }
}

int ADV::BCD_Convert(int bit8) {
  byte b[2];
  b[0] = bit8 >> 4; //shift the binary to read left most bits
  b[1] = (bit8 << 4); //shift the binary to read right most bits
  b[2] = b[1] >> 4; //shift the binary to read left most bits
  int num1 = b[0] * 10 + b[2];
  return num1;
}

// why not these for BCD conversion?
// byte bcdToDec(byte val)
// {
//   return( (val/16*10) + (val%16) );
// }
// 
// byte decToBcd(byte val)
// {
//   return( (val/10*16) + (val%10) );
// }

int ADV::s16bit(int bit8a, int bit8b) {
  int num2 = bit8a + bit8b * 256;
  if (num2 >= 32768) {
    num2 = num2 - 65536;
  }
  return num2;
}

void ADV::parseVVD(byte buf[VVDLength], double VVD[]) {//see p37 of Integration Manual for vvd structure
  //code reads until 165, 165 is not included. 165 is designator to start data packet.
  //this is why buf[0] = offset 1 in integration manual
  //buf is the datapacket offset by 1

  VVD[0] = buf[3]; //first cell in the VVD[]--> count
  int PressureMSB = buf[4];
  int PressureLSW = s16bit(buf[6], buf[7]) * 65536;
  VVD[1] = (PressureMSB + PressureLSW);//pressure msb + lsw, ask matt what this is??
  //velocity x.y.z
  VVD[2] = s16bit(buf[10], buf[11]);//x
  VVD[3] = s16bit(buf[12], buf[13]);//y
  VVD[4] = s16bit(buf[14], buf[15]);//z
  // amp
  VVD[5] = buf[16];//amplitude beam1
  VVD[6] = buf[17];
  VVD[7] = buf[18];
  //corr
  VVD[8] = buf[19];
  VVD[9] = buf[20];
  VVD[10] = buf[21];
  // AnaIn (this can't be right)
  VVD[11] = buf[2];
  // analog inputs
  VVD[12] = buf[2]+(buf[5]*256);
  VVD[13] = s16bit(buf[8], buf[9]);
}

void ADV::parseVSD(byte buf[VSDLength], double VSD[]) {
  // min, sec, day, hour, year, month
  VSD[0] = BCD_Convert(buf[4]);
  VSD[1] = BCD_Convert(buf[5]);
  VSD[2] = BCD_Convert(buf[6]);
  VSD[3] = BCD_Convert(buf[7]);
  VSD[4] = BCD_Convert(buf[8]);
  VSD[5] = BCD_Convert(buf[9]);
  // bat*0.1, soundspeed*0.1, heading*0.1, pitch*0.1, roll*0.1, temp*0.01
  VSD[6] = s16bit(buf[10], buf[11]);
  VSD[7] = s16bit(buf[12], buf[13]);
  VSD[8] = s16bit(buf[14], buf[15]);
  VSD[9] = s16bit(buf[16], buf[17]);
  VSD[10] = s16bit(buf[18], buf[19]);
  VSD[11] = s16bit(buf[20], buf[21]);
}

int ADV::getVVD() {
  if (!VVDReady) return 0;
  Serial.print("New VVD packet: ");
  for (int i = 0; i < VVDLength; ++i) {
    Serial.print(ADVpacket[i]);
    Serial.print(",");
  }
  Serial.println();

  double VVD[14];
  parseVVD(ADVpacket, VVD);
  Serial.print("New VVD data: ");
  for (int i = 0; i < 14; ++i) {
    Serial.print(VVD[i]);
    Serial.print(",");
  }
  Serial.println();
  Serial.println();
  newData = false;
  VVDReady = false;
  return 1;
}

int ADV::getVSD() {
  if (!VSDReady) return 0;
  Serial.print("New VSD packet: ");
  for (int i = 0; i < VSDLength; ++i) {
    Serial.print(ADVpacket[i]);
    Serial.print(",");
  }
  Serial.println();

  double VSD[12];
  parseVSD(ADVpacket, VSD);
  Serial.print("New VSD data: ");
  for (int i = 0; i < 12; ++i) {
    Serial.print(VSD[i]);
    Serial.print(",");
  }
  Serial.println();
  Serial.println();
  newData = false;
  VSDReady = false;
  return 1;
}

int ADV::getVVDPacket() {
  if (!VVDReady) return 0;
  Serial.print("New VVD packet: ");
  for (int i = 0; i < VVDLength; ++i) {
    Serial.print(ADVpacket[i]);
    Serial.print(",");
  }
  Serial.println();
  newData = false;
  VVDReady = false;
  return 1;
}

int ADV::getVSDPacket() {
  if (!VSDReady) return 0;
  Serial.print("New VSD packet: ");
  for (int i = 0; i < VSDLength; ++i) {
    Serial.print(ADVpacket[i]);
    Serial.print(",");
  }
  Serial.println();
  newData = false;
  VSDReady = false;
  return 1;
}



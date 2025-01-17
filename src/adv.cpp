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
    if (!validatePacket(ADVpacket, VVDLength))
    {
      Serial.println("VVD packet failed checksum");
      return;
    }
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
        ADVpacket[ndx] = rc;
        ndx++;
        if (rc == VVDChar) {
          packetLength = VVDLength;
        } else if (rc == VSDChar) {
          packetLength = VSDLength;
        } else {
          recvInProgress = false;
          ndx = 0;
        }
        
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

int ADV::calcChecksum(byte* packet, int length) {
  int checksum = 0xb58c;

  for (int i = 0; i < length; i += 2) {
    int word = packet[i] << 8 | packet[i + 1];
    checksum += word;
  }

  return checksum;
}

boolean ADV::validatePacket(byte *packet, int length) {
  // Calculate checksum
  int checksum = calcChecksum(packet, length);
  
  // Extract checksum bytes from packet
  int packetChecksum = packet[length-2] << 8 | packet[length-1]; 

  // Compare calculated vs packet checksum
  if(checksum == packetChecksum) {
    return true;
  }
  else {
    return false;
  }
}

int ADV::BCD_Convert(char bit8) {
  byte b[2];
  b[0] = bit8 >> 4; //shift the binary to read left most bits
  b[1] = (bit8 << 4); //shift the binary to read right most bits
  b[2] = b[1] >> 4; //shift the binary to read left most bits
  int num1 = b[0] * 10 + b[2];
  return num1;
}

int ADV::s16bit(char bit8a, char bit8b) {
  int num2 = bit8a + bit8b * 256;
  if (num2 >= 32768) {
    num2 = num2 - 65536;
  }
  return num2;
}

int ADV::u16bit(char bit8a, char bit8b) {
  int num2 = bit8a + bit8b * 256;
  return num2;
}

void ADV::parseVVD(byte buf[VVDLength], int VVD[]) {//see p37 of Integration Manual for vvd structure
  VVD[0] = buf[3]; // count
  // Pressure (0.001 dbar) = 65536×PressureMSB + PressureLSW
  int PressureMSB = buf[4];
  int PressureLSW = s16bit(buf[6], buf[7]);
  VVD[1] = PressureMSB * 65536 + PressureLSW;
  //velocity x.y.z (mm/s)
  VVD[2] = s16bit(buf[10], buf[11]);//x
  VVD[3] = s16bit(buf[12], buf[13]);//y
  VVD[4] = s16bit(buf[14], buf[15]);//z
  // amplitude
  VVD[5] = buf[16];
  VVD[6] = buf[17];
  VVD[7] = buf[18];
  //correlation (0-100%)
  VVD[8] = buf[19];
  VVD[9] = buf[20];
  VVD[10] = buf[21];
  VVD[11] = u16bit(buf[8], buf[9]); // Analog in 1, unsigned for pH
  VVD[12] = s16bit(buf[2], buf[5]); // analog in 2
  VVD[13] = s16bit(buf[22], buf[23]); //checksum
}

void ADV::parseVSD(byte buf[VSDLength], int VSD[]) {
  // min, sec, day, hour, year, month
  VSD[0] = BCD_Convert(buf[4]);
  VSD[1] = BCD_Convert(buf[5]);
  VSD[2] = BCD_Convert(buf[6]);
  VSD[3] = BCD_Convert(buf[7]);
  VSD[4] = BCD_Convert(buf[8]);
  VSD[5] = BCD_Convert(buf[9]);
  VSD[6] = s16bit(buf[10], buf[11]); // bat*0.1V
  VSD[7] = s16bit(buf[12], buf[13]); // soundspeed*0.1m/s
  VSD[8] = s16bit(buf[14], buf[15]); // heading*0.1deg
  VSD[9] = s16bit(buf[16], buf[17]); // pitch*0.1deg
  VSD[10] = s16bit(buf[18], buf[19]); // roll*0.1deg
  VSD[11] = s16bit(buf[20], buf[21]); // temp*0.01degC
  VSD[12] = buf[22]; // error byte
  VSD[13] = buf[23]; // status byte
  VSD[14] = s16bit(buf[24], buf[25]); // Analog input
  VSD[15] = s16bit(buf[26], buf[27]); // checksum
}

int ADV::getVVD() {
  if (!VVDReady) return 0;
  int VVD[14];
  parseVVD(ADVpacket, VVD);
  char buf[128];
  int n = snprintf(buf, sizeof(buf), "D:%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
    VVD[0], VVD[1], VVD[2], VVD[3], VVD[4], VVD[5], VVD[6], VVD[7],
    VVD[8], VVD[9], VVD[10], VVD[11], VVD[12], VVD[13]);
  Serial.write(buf, n);
  newData = false;
  VVDReady = false;
  return 1;
}

int ADV::getVSD() {
  if (!VSDReady) return 0;
  int VSD[16];
  parseVSD(ADVpacket, VSD);
  char buf[128];
  int n = snprintf(buf, sizeof(buf), "S:%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
    VSD[0], VSD[1], VSD[2], VSD[3], VSD[4], VSD[5], VSD[6], VSD[7],
    VSD[8], VSD[9], VSD[10], VSD[11], VSD[12], VSD[13], VSD[14], VSD[15]);
  Serial.write(buf, n);
  newData = false;
  VSDReady = false;
  return 1;
}

int ADV::getVVDPacket() {
  if (!VVDReady) return 0;
  for (int i = 0; i < VVDLength; ++i) {
    Serial.write(ADVpacket[i]);
  }
  newData = false;
  VVDReady = false;
  return 1;
}

int ADV::getVSDPacket() {
  if (!VSDReady) return 0;
  for (int i = 0; i < VSDLength; ++i) {
    Serial.write(ADVpacket[i]);
  }
  newData = false;
  VSDReady = false;
  return 1;
}
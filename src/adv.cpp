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

void ADV::begin()
{
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
    // newData = false;
  }
}

void ADV::read_serial()
{
  static byte ndx = 0;
  static boolean recvInProgress = false;
  static byte packetLength;
  byte rc;
  while (serial.available() > 0 && newData == false)
  {
    rc = serial.read();
    if (recvInProgress == true)
    {
      if (ndx == 1)
      {
        if (rc == VVDChar)
        {
          packetLength = VVDLength;
        }
        else
        {
          packetLength = VSDLength;
        }
        ADVpacket[ndx] = rc;
        ndx++;
      }
      else if (ndx == packetLength - 1)
      { // whole packet received
        ADVpacket[ndx] = rc;
        ndx++;
        ADVpacket[ndx] = '\0';
        ndx = 0;
        newData = true;
        recvInProgress = false;
      }
      else
      {
        ADVpacket[ndx] = rc;
        ndx++;
      }
    }
    else if (rc == startMarker)
    {
      ADVpacket[ndx] = rc;
      ndx++;
      recvInProgress = true;
    }
  }
}

int ADV::BCD_Convert(int bit8)
{
  byte b[2];
  b[0] = bit8 >> 4;   // shift the binary to read left most bits
  b[1] = (bit8 << 4); // shift the binary to read right most bits
  b[2] = b[1] >> 4;   // shift the binary to read left most bits
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

int ADV::s16bit(int bit8a, int bit8b)
{
  int num2 = bit8a + bit8b * 256;
  if (num2 >= 32768)
  {
    num2 = num2 - 65536;
  }
  return num2;
}

void ADV::parseVVD(byte buf[VVDLength], VectorVelocityData vvd)
{
  // see p37 of Integration Manual for vvd structure
  vvd.Sync = buf[0];
  vvd.Id = buf[1];
  vvd.Count = buf[3];
  vvd.Pressure = buf[4] * 65536 + (buf[6] + buf[7] * 256);
  vvd.AnaIn1 = buf[8] + buf[9] * 256;
  vvd.AnaIn2 = buf[2] + buf[5] * 256;
  // amp
  vvd.Amplitude[0] = buf[16]; // amplitude beam1
  vvd.Amplitude[0] = buf[17];
  vvd.Amplitude[2] = buf[18];
  // corr
  vvd.Correlation[0] = buf[19];
  vvd.Correlation[1] = buf[20];
  vvd.Correlation[2] = buf[21];
  // velocity x.y.z
  vvd.Velocity[0] = s16bit(buf[10], buf[11]); // x
  vvd.Velocity[1] = s16bit(buf[12], buf[13]); // y
  vvd.Velocity[2] = s16bit(buf[14], buf[15]); // z
  vvd.Checksum = s16bit(buf[22], buf[23]);
}

void ADV::parseVSD(byte buf[VSDLength], VectorSystemData vsd)
{
  vsd.Sync = buf[0];
  vsd.Id = buf[1];
  vsd.Size = buf[2] + buf[3] * 256;
  vsd.Time.Year = BCD_Convert(buf[8]);
  vsd.Time.Month = BCD_Convert(buf[9]);
  vsd.Time.Day = BCD_Convert(buf[6]);
  vsd.Time.Hour = BCD_Convert(buf[7]);
  vsd.Time.Minute = BCD_Convert(buf[4]);
  vsd.Time.Second = BCD_Convert(buf[5]);
  vsd.Battery = buf[10] + buf[11] * 256;
  vsd.Soundspeed = buf[12] + buf[13] * 256;
  vsd.Heading = s16bit(buf[14], buf[15]);
  vsd.Pitch = s16bit(buf[16], buf[17]);
  vsd.Roll = s16bit(buf[18], buf[19]);
  vsd.Temperature = s16bit(buf[20], buf[21]);
  vsd.Error = buf[22];
  vsd.Status = buf[23];
  vsd.AnaIn = buf[24] + buf[25] * 256;
  vsd.Checksum = s16bit(buf[26], buf[27]);
}

int ADV::getVVD()
{
  if (!VVDReady)
    return 0;
  Serial.print("New VVD packet: ");
  for (int i = 0; i < VVDLength; ++i)
  {
    Serial.print(ADVpacket[i]);
    Serial.print(",");
  }
  Serial.println();

  VectorVelocityData vvd;
  parseVVD(ADVpacket, vvd);
  Serial.print("New VVD data: ");
  char buffer[256];
  int pos = 0;
  pos += snprintf(buffer + pos, sizeof(buffer) - pos, "%d,%d,%d,%d,%d,%d,",
                  vvd.Sync, vvd.Id, vvd.Count, vvd.Pressure, vvd.AnaIn1, vvd.AnaIn2);

  for (int i = 0; i < 3; i++)
  {
    pos += snprintf(buffer + pos, sizeof(buffer) - pos, "%d,", vvd.Amplitude[i]);
  }
  for (int i = 0; i < 3; i++)
  {
    pos += snprintf(buffer + pos, sizeof(buffer) - pos, "%d,", vvd.Correlation[i]);
  }
  for (int i = 0; i < 3; i++)
  {
    pos += snprintf(buffer + pos, sizeof(buffer) - pos, "%d,", vvd.Velocity[i]);
  }
  pos += snprintf(buffer + pos, sizeof(buffer) - pos, "%d", vvd.Checksum);
  Serial.println(buffer);
  Serial.println();
  newData = false;
  VVDReady = false;
  return 1;
}

int ADV::getVSD()
{
  if (!VSDReady)
    return 0;
  Serial.print("New VSD packet: ");
  for (int i = 0; i < VSDLength; ++i)
  {
    Serial.print(ADVpacket[i]);
    Serial.print(",");
  }
  Serial.println();

  VectorSystemData vsd;
  parseVSD(ADVpacket, vsd);
  Serial.print("New VSD data: ");
  char buffer[256];
  int pos = 0;
  pos += snprintf(buffer + pos, sizeof(buffer) - pos, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
          vsd.Sync, vsd.Id, vsd.Size,
          vsd.Time.Year, vsd.Time.Month, vsd.Time.Day,
          vsd.Time.Hour, vsd.Time.Minute, vsd.Time.Second,
          vsd.Battery, vsd.Soundspeed, vsd.Heading,
          vsd.Pitch, vsd.Roll, vsd.Temperature,
          vsd.Error, vsd.Status, vsd.Checksum);
  Serial.println();
  newData = false;
  VSDReady = false;
  return 1;
}

int ADV::getVVDPacket()
{
  if (!VVDReady)
    return 0;
  Serial.print("New VVD packet: ");
  for (int i = 0; i < VVDLength; ++i)
  {
    Serial.print(ADVpacket[i]);
    Serial.print(",");
  }
  Serial.println();
  newData = false;
  VVDReady = false;
  return 1;
}

int ADV::getVSDPacket()
{
  if (!VSDReady)
    return 0;
  Serial.print("New VSD packet: ");
  for (int i = 0; i < VSDLength; ++i)
  {
    Serial.print(ADVpacket[i]);
    Serial.print(",");
  }
  Serial.println();
  newData = false;
  VSDReady = false;
  return 1;
}

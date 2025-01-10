# Nortek Vector ADV Arduino Library

This library allows control and data transfer for Nortek Vector ADV's via
serial from the Arduino platform. Tested with Teensy 4.1 and the teensyduino
framework.

# Use

The library provides an ADV class with functions for initializing the ADV,
setting the ADV to stream data via serial, and for reading and parsing 
status and data packets.

To enable fast, non-blocking reading of data, `ADV.read()` should run in a 
`loop()` that runs many times per second. `ADV.getVVDPacket()` and 
`ADV.getVSDPacket()` will poll for a complete, parsed packet and return 0 if 
a new packet is not ready.
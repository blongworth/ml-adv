/** Read Nortec Vector ADV serial packets */

#include <Arduino.h>
#include "adv.h"

ADV adv;

void setup() {
  Serial.begin(9600);
  delay(3000);
  Serial.println("Teensy ready");
  Serial.println("Starting ADV...");
  adv.begin();
}

void loop() {
  adv.read();
  adv.getVVDPacket();
  adv.getVSDPacket();
}
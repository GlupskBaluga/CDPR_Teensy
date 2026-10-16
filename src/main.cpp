#include <Arduino.h>
#include "EthernetManager.h"

EthernetManager ethernet;


void setup() {
    Serial.begin(115200);
    delay(2000);

    ethernet.begin();
}

void loop() {
    ethernet.printLinkStatus();
    
    delay(1000);
}
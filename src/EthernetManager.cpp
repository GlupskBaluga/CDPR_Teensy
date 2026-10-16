#include <Arduino.h>
#include <NativeEthernet.h>

#include "EthernetManager.h"

namespace {
    byte mac[] = {0x02, 0xE9, 0xE5, 0x41, 0x00, 0x01};

    IPAddress ip(192, 168, 10, 68);
    IPAddress dns(192, 168, 10, 1);
    IPAddress gateway(192, 168, 10, 1);
    IPAddress subnet(255, 255, 255, 0);
}

void EthernetManager::begin() {
    Serial.println("Initializing Ethernet...");

    Ethernet.begin(mac, ip, dns, gateway, subnet);
    delay(1000);

    Serial.print("IP: ");
    Serial.println(Ethernet.localIP());
}

void EthernetManager::printLinkStatus() {
    auto status = Ethernet.linkStatus();

    if (status == LinkON) {
        Serial.println("LINK UP");
    } else if (status == LinkOFF) {
        Serial.println("LINK DOWN");
    } else {
        Serial.println("LINK UNKNOWN");
    }
}
// #include <Arduino.h>
// #include <string>
// Bluetooth用設定
#include "BluetoothSerial.h"
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
BluetoothSerial SerialBT_;
#include "Esp32BT.h"

// EspBTSetUpClass::EspBTSetUpClass() {}

void EspBTSetUpClass::getMacAddress(){
    uint8_t macBT[6];
    esp_read_mac(macBT, ESP_MAC_BT);
    Serial.print("MyMacAddress:");
    Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X\r\n", macBT[0], macBT[1], macBT[2], macBT[3], macBT[4], macBT[5]);
}

void EspBTSetUpClass::mainUnitSetUp(uint8_t address[6]){
    bool connected;
    // Bluetoothのローカルデバイス名の設定第2引数でマスター側の設定trueでマスター
    SerialBT_.begin("ESP32", true); 
    connected = SerialBT_.connect(address);
    if(connected) {
        Serial.println("Connect OK");
    } else {
        while(!SerialBT_.connected(1500)) {
        connected = SerialBT_.connect(address);
        Serial.println("No connect"); 
        }
    }
    if (SerialBT_.disconnect()) {
        Serial.println("Disconnected Succesfully!");
    }
    SerialBT_.connect();
}

void EspBTSetUpClass::childUnitSetUp(){
    SerialBT_.begin("ESP32");
}
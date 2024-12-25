#ifndef ESP32BT_H
#define ESP32BT_H
#include <Arduino.h>
#include <string>
// Bluetooth用設定
#include "BluetoothSerial.h"
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
/*
親機設定方法
setup()関数に記載
    #include "Secret.h"//通信ユニットのmac address記載
    String MACadd = MAC_ADDRESS_STR;
    uint8_t address[6] = {MAC_ADDRESS_BYTE[0], MAC_ADDRESS_BYTE[1], MAC_ADDRESS_BYTE[2], MAC_ADDRESS_BYTE[3], MAC_ADDRESS_BYTE[4], MAC_ADDRESS_BYTE[5]};
    EspBTSetUpClass EspBTSetObj;
    EspBTSetObj.getMacAddress();
    EspBTSetObj.mainUnitSetUp(address);
子機設定方法
setup()関数に記載
    EspBTSetUpClass EspBTSetObj;
    EspBTSetObj.getMacAddress();
    EspBTSetObj.childUnitSetUp();
*/

class EspBTSetUpClass{
public:
    EspBTSetUpClass() {
        Serial.println("ESP32 Bluetooth set up class");
    }

    void getMacAddress();
    void mainUnitSetUp(uint8_t address[6]);
    void childUnitSetUp();
};

#endif
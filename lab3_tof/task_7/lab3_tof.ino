/*
  Lab 3 - ToF Sensor with BLE
  VL53L1X distance sensor in Long mode, streamed over BLE to Jupyter Lab.
*/

#include <Wire.h>
#include "SparkFun_VL53L1X.h"

#define SHUTDOWN_PIN 2
#define INTERRUPT_PIN 3

SFEVL53L1X distanceSensor; 

//////////// BLE Libraries ////////////
#include "BLECStringCharacteristic.h"
#include "EString.h"
#include "RobotCommand.h"
#include <ArduinoBLE.h>

//////////// BLE UUIDs ////////////
#define BLE_UUID_TEST_SERVICE "15bb5de7-5941-4ba2-bda0-784bb8817a1b"
#define BLE_UUID_RX_STRING    "9750f60b-9c9c-4158-b620-02ec9521cd99"
#define BLE_UUID_TX_FLOAT     "27616294-3063-4ecc-b60b-3470ddef2938"
#define BLE_UUID_TX_STRING    "f235a225-6735-4d73-94cb-ee5dfce9ba83"

//////////// BLE Globals ////////////
BLEService testService(BLE_UUID_TEST_SERVICE);
BLECStringCharacteristic rx_characteristic_string(BLE_UUID_RX_STRING, BLEWrite, MAX_MSG_SIZE);
BLEFloatCharacteristic  tx_characteristic_float(BLE_UUID_TX_FLOAT, BLERead | BLENotify);
BLECStringCharacteristic tx_characteristic_string(BLE_UUID_TX_STRING, BLERead | BLENotify, MAX_MSG_SIZE);

RobotCommand robot_cmd(":|");
EString tx_estring_value;

// Declare distance as a global
int distance = 0;

//////////// Commands ////////////
enum CommandTypes {
    GET_SENSOR1_DATA,
};

void handle_command() {
    robot_cmd.set_cmd_string(rx_characteristic_string.value(),
                             rx_characteristic_string.valueLength());

    bool success;
    int cmd_type = -1;
    success = robot_cmd.get_command_type(cmd_type);
    if (!success) return;

    switch (cmd_type) {
        case GET_SENSOR1_DATA:
            tx_estring_value.clear();
            tx_estring_value.append("Distance(mm):");
            tx_estring_value.append(distance);
            tx_estring_value.append(",T:");
            tx_estring_value.append((int)millis());
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
            break;

        default:
            Serial.print("Invalid Command Type: ");
            Serial.println(cmd_type);
            break;
    }
}

//////////// Setup ////////////
void setup(void) {
    Wire.begin();
    Serial.begin(115200);
    Serial.println("VL53L1X Qwiic Test");

    //////////// BLE Setup ////////////
    BLE.begin();
    BLE.setDeviceName("Artemis BLE");
    BLE.setLocalName("Artemis BLE");
    BLE.setAdvertisedService(testService);
    testService.addCharacteristic(tx_characteristic_float);
    testService.addCharacteristic(tx_characteristic_string);
    testService.addCharacteristic(rx_characteristic_string);
    BLE.addService(testService);
    tx_characteristic_float.writeValue(0.0);
    tx_estring_value.clear();
    tx_estring_value.append("[->"); tx_estring_value.append(9.0); tx_estring_value.append("<-]");
    tx_characteristic_string.writeValue(tx_estring_value.c_str());
    Serial.print("Advertising BLE with MAC: ");
    Serial.println(BLE.address());
    BLE.advertise();

    //////////// ToF Setup ////////////
    if (distanceSensor.begin() != 0) {
        Serial.println("Sensor failed to begin. Please check wiring. Freezing...");
        while (1);
    }
    // Long mode, defaults, but just in case, call explicitly:
    distanceSensor.setDistanceModeLong(); 
}

//////////// Loop ////////////
void loop(void) {
    BLEDevice central = BLE.central();

    if (central) {
        Serial.print("Connected to: ");
        Serial.println(central.address());

        while (central.connected()) {
            // Continuously take a fresh ranging measurement
            distanceSensor.startRanging();
            while (!distanceSensor.checkForDataReady()) {
                delay(1);
            }
            distance = distanceSensor.getDistance(); // updates global
            distanceSensor.clearInterrupt();
            distanceSensor.stopRanging();

            Serial.print("Distance(mm): ");
            Serial.println(distance);

            // Respond to any incoming BLE command
            if (rx_characteristic_string.written()) {
                handle_command();
            }
        }

        Serial.println("Disconnected from central");
    }
}

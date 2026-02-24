/*
  Lab 3 - ToF Sensor with BLE
  VL53L1X distance sensor in Long mode, streamed over BLE to Jupyter Lab.
*/

#include <Wire.h>
#include "SparkFun_VL53L1X.h"

// Only sensor 2 has XSHUT, connected to Artemis Nano A0 pin
#define XSHUT_Sensor2 A0

// Two sensor objects
SFEVL53L1X sensor1;
SFEVL53L1X sensor2;

// Sensor 1 gets a new address
#define SENSOR1_ADDR 0x30
// Sensor 2 stays at default 0x29

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

// Declare distances as globals
int distance1 = 0;
int distance2 = 0;

//////////// Commands ////////////
enum CommandTypes {
    GET_TOF_DATA,
};

void handle_command() {
    robot_cmd.set_cmd_string(rx_characteristic_string.value(),
                             rx_characteristic_string.valueLength());

    bool success;
    int cmd_type = -1;
    success = robot_cmd.get_command_type(cmd_type);
    if (!success) return;

    switch (cmd_type) {
        case GET_TOF_DATA:
            tx_estring_value.clear();
            tx_estring_value.append("Distance_sensor1(mm):");
            tx_estring_value.append(distance1);
            tx_estring_value.append(",Distance_sensor2(mm):");  
            tx_estring_value.append(distance2);
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
    // for two sensors
    pinMode(XSHUT_Sensor2, OUTPUT);
    digitalWrite(XSHUT_Sensor2, LOW); // sensor 2 off, sensor 1 on
    delay(10);
    // init and reassign address
    if (sensor1.begin() != 0) {
        Serial.println("Sensor 1 failed to begin. Check wiring. Freezing...");
        while (1);
    }
    // move sensor 1 to new address 0x30
    sensor1.setI2CAddress(SENSOR1_ADDR);
    Serial.println("Sensor 1 at 0x30");
    // wake up sensor 2 --> no other sensor at address 0x29, so no conflict
    digitalWrite(XSHUT_Sensor2, HIGH);
    delay(10);
    if (sensor2.begin() != 0) {
        Serial.println("Sensor 2 failed to begin. Check wiring. Freezing...");
        while (1);
    }
    Serial.println("Sensor 2 at 0x29");
    // default mode long, so no extra call needed, but just in case, call explicitly:
    sensor1.setDistanceModeLong();
    sensor2.setDistanceModeLong();
    Serial.println("Both sensors ready!");
}

//////////// Loop ////////////
void loop(void) {
    BLEDevice central = BLE.central();

    if (central) {
        Serial.print("Connected to: ");
        Serial.println(central.address());

        while (central.connected()) {
            // Continuously take a fresh ranging measurement from both sensors
            distance1 = readSensor(sensor1);
            distance2 = readSensor(sensor2);

            Serial.print("Distance_sensor1(mm): ");
            Serial.println(distance1);
            Serial.print("Distance_sensor2(mm): ");
            Serial.println(distance2);

            // Respond to any incoming BLE command
            if (rx_characteristic_string.written()) {
                handle_command();
            }
        }

        Serial.println("Disconnected from central");
    }
}

// helper function, read sensor distance
int readSensor(SFEVL53L1X &sensor) {
    sensor.startRanging();
    while (!sensor.checkForDataReady()) {
        delay(1);
    }
    int d = sensor.getDistance();
    sensor.clearInterrupt();
    sensor.stopRanging();
    return d;
}

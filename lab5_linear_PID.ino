// Lab 5 - Linear PID control and Linear interpolation
// this test program tests the car's ability to move forward for about 1m, backward for about 1m, rotate CCW for 180 degrees, and rotate CCW for 360 degrees. 

////////////////////////////////////////////////////////// Lab 4 - Motor Control //////////////////////////////////
// Motor 1 (right, front view) pins
#define MOTOR1_IN_PLUS 12
#define MOTOR1_IN_MINUS A15
#define DEADBAND_1 80

// Motor 2 (left) pins
#define MOTOR2_IN_PLUS A2
#define MOTOR2_IN_MINUS 4
#define DEADBAND_2 130
#define CALIBRATION_FACTOR 1.1 

int pwm_speed = 150; // PWM value 0-255
int forward_1m_time = 1000;
int backward_1m_time = 1000;
int rotate_180_CCW_time = 1300;
int rotate_360_CW_time = 2600;

int applyDeadband(int speed, int deadband) {
  if (speed == 0) {
    return 0;
  } else {
    // else, map pwm speed 0-255 to deadband (new starting speed) -255
    return map(speed, 1, 255, deadband, 255);
  }
}

// movement functions
void forward(int speed) {
  analogWrite(MOTOR1_IN_PLUS, applyDeadband(speed, DEADBAND_1)*CALIBRATION_FACTOR);
  analogWrite(MOTOR1_IN_MINUS, 0);
  analogWrite(MOTOR2_IN_PLUS, applyDeadband(speed, DEADBAND_2));
  analogWrite(MOTOR2_IN_MINUS, 0);
}

void backward(int speed) {
  analogWrite(MOTOR1_IN_PLUS, 0);
  analogWrite(MOTOR1_IN_MINUS, applyDeadband(speed, DEADBAND_1)*CALIBRATION_FACTOR);
  analogWrite(MOTOR2_IN_PLUS, 0);
  analogWrite(MOTOR2_IN_MINUS, applyDeadband(speed, DEADBAND_2));
}

void stop() {
  analogWrite(MOTOR1_IN_PLUS, 0);
  analogWrite(MOTOR1_IN_MINUS, 0);
  analogWrite(MOTOR2_IN_PLUS, 0);
  analogWrite(MOTOR2_IN_MINUS, 0);
}

void rotateCCW(int speed) {
 analogWrite(MOTOR1_IN_PLUS, 0);
 analogWrite(MOTOR1_IN_MINUS, applyDeadband(speed, DEADBAND_1)*CALIBRATION_FACTOR);
 analogWrite(MOTOR2_IN_PLUS, applyDeadband(speed, DEADBAND_2));
 analogWrite(MOTOR2_IN_MINUS, 0);
}

void rotateCW(int speed) {
 analogWrite(MOTOR1_IN_PLUS, applyDeadband(speed, DEADBAND_1)*CALIBRATION_FACTOR);
 analogWrite(MOTOR1_IN_MINUS, 0);
 analogWrite(MOTOR2_IN_PLUS, 0);
 analogWrite(MOTOR2_IN_MINUS, applyDeadband(speed, DEADBAND_2));
}
////////////////////////////////////////////////////////// Lab 3 - ToF Sensor Reading //////////////////////////////////
#include <Wire.h>
#include "SparkFun_VL53L1X.h"
// ToF Declarations & Variables
// Two sensor objects
SFEVL53L1X sensor1;
SFEVL53L1X sensor2;

// Only sensor 2 has XSHUT, connected to Artemis Nano A0 pin
#define XSHUT_Sensor2 A0
// Sensor 1 gets a new address
#define SENSOR1_ADDR 0x30
// Sensor 2 stays at default 0x29

// Declare distances as globals
int distance1 = 0;
int distance2 = 0;

////////////////////////////////////////////////////////// Lab 1 - BLE //////////////////////////////////
// For sending data over ble to python 
// Debugging data
// ToF
const int ARRAY_SIZE = 100;
unsigned long Distance1_arr[ARRAY_SIZE]; 
unsigned long Distance2_arr[ARRAY_SIZE]; 
// IMU (for lab 6, orientation control)
unsigned long Roll_arr[ARRAY_SIZE]; 
unsigned long Pitch_arr[ARRAY_SIZE]; 
unsigned long Yaw_arr[ARRAY_SIZE]; 
// Timestamp
unsigned long T_arr[ARRAY_SIZE]; 
unsigned long millisTime;
// BLE libraries
#include "BLECStringCharacteristic.h"
#include "EString.h"
#include "RobotCommand.h"
#include <ArduinoBLE.h>
// BLE UUIDs
#define BLE_UUID_TEST_SERVICE "15bb5de7-5941-4ba2-bda0-784bb8817a1b"
#define BLE_UUID_RX_STRING "9750f60b-9c9c-4158-b620-02ec9521cd99"
#define BLE_UUID_TX_FLOAT "27616294-3063-4ecc-b60b-3470ddef2938"
#define BLE_UUID_TX_STRING "f235a225-6735-4d73-94cb-ee5dfce9ba83"
// BLE Globals
BLEService testService(BLE_UUID_TEST_SERVICE);
BLECStringCharacteristic rx_characteristic_string(BLE_UUID_RX_STRING, BLEWrite, MAX_MSG_SIZE);
BLEFloatCharacteristic  tx_characteristic_float(BLE_UUID_TX_FLOAT, BLERead | BLENotify);
BLECStringCharacteristic tx_characteristic_string(BLE_UUID_TX_STRING, BLERead | BLENotify, MAX_MSG_SIZE);

////////////////////////////////////////////////////////// RobotCommand, communication b/w Artemis & computer //////////////////////////////////
RobotCommand robot_cmd(":|");
EString tx_estring_value;
// Commands
enum CommandTypes {
    GET_2TOF_DATA,
    GET_IMU_DATA,
    GET_2TOF_IMU_TIME_DATA,
};
// Case statement to handle commands
void handle_command() {
    robot_cmd.set_cmd_string(rx_characteristic_string.value(),
                             rx_characteristic_string.valueLength());

    bool success;
    int cmd_type = -1;
    success = robot_cmd.get_command_type(cmd_type);
    if (!success) return;

    switch (cmd_type) {
        case GET_2TOF_DATA:
            millisTime = millis();
            tx_estring_value.clear();
            tx_estring_value.append("Distance1:");
            tx_estring_value.append(distance1);
            tx_estring_value.append("Distance2:");
            tx_estring_value.append(distance2);
            tx_estring_value.append("T:");
            tx_estring_value.append((int)millisTime);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
            Serial.print("Send to Python: ");
            Serial.println(tx_estring_value.c_str());

            break;

        case GET_IMU_DATA:
            millisTime = millis();
            tx_estring_value.clear();
            tx_estring_value.append("T:");
            tx_estring_value.append((int)millisTime);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
            Serial.print("Send to Python: ");
            Serial.println(tx_estring_value.c_str());

            break;
        
        case GET_2TOF_IMU_TIME_DATA:
            millisTime = millis();
            tx_estring_value.clear();
            tx_estring_value.append("T:");
            tx_estring_value.append((int)millisTime);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
            Serial.print("Send to Python: ");
            Serial.println(tx_estring_value.c_str());

            break;

        default:
            Serial.print("Invalid Command Type: ");
            Serial.println(cmd_type);
            break;
    }
}
////////////////////////////////////////////////////////// Setup //////////////////////////////////


void setup() {
  Serial.begin(115200);
  Wire.begin(); // start I2C
  Serial.println("VL53L1X Qwiic Test");
  ////////////// BLE Setup /////////////////////////////
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
  ////////////// 2 ToF Sensor Setup /////////////////////////////
  pinMode(XSHUT_Sensor2, OUTPUT);
  digitalWrite(XSHUT_Sensor2, LOW); // sensor 2 off, sensor 1 on
  delay(100); // w/o this, Sensor 1 always fail to begin, wait for shutdown of sensor 2 to complete
  // init and reassign address
  if (sensor1.begin() != 0) {
      Serial.println("Sensor 1 failed to begin. Please check wiring. Freezing...");
      while (1);
  }
  // move sensor 1 to new address 0x30
  sensor1.setI2CAddress(SENSOR1_ADDR);
  Serial.println("Sensor 1 at 0x30");
  // wake up sensor 2 --> no other sensor at address 0x29, so no conflict
  digitalWrite(XSHUT_Sensor2, HIGH);
  delay(100);
  if (sensor2.begin() != 0) {
      Serial.println("Sensor 2 failed to begin. Check wiring. Freezing...");
      while (1);
  }
  Serial.println("Sensor 2 at 0x29");
  // default mode long, so no extra call needed, but just in case, call explicitly:
  sensor1.setDistanceModeLong();
  sensor2.setDistanceModeLong();
  Serial.println("Both sensors ready!");
  ////////////// 2 Motor Setup ////////////////////////////////////
  // Set as output (from Artemis, input to motor controller)
  pinMode(MOTOR1_IN_PLUS, OUTPUT);
  pinMode(MOTOR1_IN_MINUS, OUTPUT);
  pinMode(MOTOR2_IN_PLUS, OUTPUT);
  pinMode(MOTOR2_IN_MINUS, OUTPUT);
}

////////////////////////////////////////////////////////// Main loop //////////////////////////////////
// Sample ToF sensor reading data
// Execute motor control using pwm_speed
void loop() {
  BLEDevice central = BLE.central();
  if (central) {
    Serial.print("Connected to: ");
    Serial.println(central.address());
    while (central.connected()) { // Hardstop --> if ble connection lost, motor stop (pwm_speed = 0)
      // Continuously take a fresh ranging measurement
      sensor1.startRanging();
      sensor2.startRanging();
      while (!sensor1.checkForDataReady()) {
          delay(1);
      }
      while (!sensor2.checkForDataReady()) {
          delay(1);
      }
      distance1 = sensor1.getDistance(); // updates global
      distance2 = sensor2.getDistance(); // updates global
      sensor1.clearInterrupt();
      sensor2.clearInterrupt();
      sensor1.stopRanging();
      sensor2.stopRanging();

      Serial.print("Distance1(mm):");
      Serial.println(distance1);
      Serial.print("Distance2(mm):");
      Serial.println(distance2);

      // Respond to any incoming BLE command
      if (rx_characteristic_string.written()) {
          handle_command();
      }
    }
    Serial.println("Disconnected from central");
  }
}


/*
// stop for 10 second
  Serial.println("Stop!");
  stop();
  delay(10000); 
  // move forward 
  Serial.println("Move forward");
  forward(pwm_speed);
  delay(forward_1m_time);

  // stop for 3 second
  Serial.println("Stop!");
  stop();
  delay(3000); 

  // move backward
  Serial.println("Move backward");
  backward(pwm_speed);
  delay(backward_1m_time);

  // stop for 3 second
  Serial.println("Stop!");
  stop();
  delay(3000);  

  // Rotate CCW 180
  Serial.println("Rotate CCW 180°");
  rotateCW(pwm_speed);
  delay(rotate_180_CCW_time);

  // Stop for 3 seconds
  Serial.println("Stop!");
  stop();
  delay(3000); 

  // Rotate CW 360°
  Serial.println("Rotate CW 360°");
  rotateCW(pwm_speed);
  delay(rotate_360_CW_time);
  
  // Stop for 3 seconds
  Serial.println("Stop!");
  stop();
  delay(30000); */







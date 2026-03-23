// Lab 5 - Linear PID control and Linear interpolation
// this test program tests the car's ability to move forward as fast as possible, and stop 1ft (304mm) away from wall

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
SFEVL53L1X sensor1; // front
SFEVL53L1X sensor2; // side

// Only sensor 2 has XSHUT, connected to Artemis Nano A0 pin
#define XSHUT_Sensor2 A0
// Sensor 1 gets a new address
#define SENSOR1_ADDR 0x30
// Sensor 2 stays at default 0x29

// Declare distances as globals
// Measured distance
int distance1 = 0;
int distance2 = 0;

////////////////////////////////////////////////////////// Data Collection //////////////////////////////////
// For sending data over ble to python 
// Debugging data
// ToF
const int ARRAY_SIZE = 500;
int Measured_distance_arr[ARRAY_SIZE]; 
int Error_arr[ARRAY_SIZE]; 
int Control_speed_arr[ARRAY_SIZE]; 
// IMU (for lab 6, orientation control)
unsigned long Roll_arr[ARRAY_SIZE]; 
unsigned long Pitch_arr[ARRAY_SIZE]; 
unsigned long Yaw_arr[ARRAY_SIZE]; 
// Timestamp
unsigned long T_arr[ARRAY_SIZE]; 
// Array index
int arr_index = 0;

////////////////////////////////////////////////////////// Lab 1 - BLE //////////////////////////////////
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

////////////////////////////////////////////////////////// Lab 5 - Linear PID //////////////////////////////////
#define LINEAR_SETPOINT 304 // 304mm = 1ft, expected distance
#define MIN_SPEED 0
#define MAX_SPEED 255
float linear_Kp = 0.5;
float linear_Ki = 0.0;
float linear_Kd = 0.0;
int linear_error = 0;
int linear_sum_error = 0;
int linear_previous_error = 0;
int linear_derivative_error = 0;
int linear_control_speed = 0;
bool linear_pid_running = false;

// Linear PID Controller w/ only proportional term (Kp)
void runLinearPIDController() {
  // Continuously take a fresh ranging measurement
  // for liner PID, use sensor 1 only
  sensor2.startRanging();
  while (!sensor2.checkForDataReady()) {
      delay(1);
  }
  distance2 = sensor2.getDistance(); // updates global
  sensor2.clearInterrupt();
  sensor2.stopRanging();

  linear_error = distance2 - LINEAR_SETPOINT;
  linear_control_speed = (int)(linear_Kp*abs(linear_error)); // comment out when add I & D
  
  /* for I & D
  linear_sum_error = linear_sum_error + linear_error;
  linear_derivative_error = linear_error - linear_previous_error;
  linear_control_speed = (int)(linear_Kp*linear_error + linear_Ki*linear_sum_error + linear_Kd*linear_derivative_error);
  linear_previous_error = linear_error;
  */

  linear_control_speed = constrain(linear_control_speed, MIN_SPEED, MAX_SPEED);

  ////////////// Collect data ////////////////////////////////////
  if (arr_index < ARRAY_SIZE) {
    T_arr[arr_index] = millis();
    Measured_distance_arr[arr_index] = distance2;
    Error_arr[arr_index] = linear_error;
    Control_speed_arr[arr_index] = linear_control_speed;
    arr_index++;
  }
  
  if (abs(linear_error) < 20) { // comment out when add I & D
    stop();
  } else if (linear_error > 0) {
    forward(linear_control_speed);
  } else {
    backward(linear_control_speed);
  }
}

////////////////////////////////////////////////////////// RobotCommand, communication b/w Artemis & computer //////////////////////////////////
RobotCommand robot_cmd(":|");
EString tx_estring_value;
// Commands
enum CommandTypes {
    START_LINEAR_PID,
    STOP_LINEAR_PID,
    SEND_LINEAR_PID_DATA,
    SET_LINEAR_PID_GAIN,
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
      case START_LINEAR_PID:
        Serial.println("Start linear PID!");
        linear_pid_running = true;
        break;

      case STOP_LINEAR_PID:
        Serial.println("Stop linear PID!");
        linear_pid_running = false;
        stop(); // stop motors
        break;

      case SEND_LINEAR_PID_DATA:
        Serial.println("Sending debugging data!");
        for (int i = 0; i < arr_index; i++) {
          tx_estring_value.clear();

          tx_estring_value.append("T:"); // timestamp
          tx_estring_value.append((int)T_arr[i]);
          tx_estring_value.append("|D:"); // measured distance from ToF sensor reading
          tx_estring_value.append(Measured_distance_arr[i]);
          tx_estring_value.append("|E:"); // error (distance difference of expected/desire and measured distance)
          tx_estring_value.append(Error_arr[i]);
          tx_estring_value.append("|C:"); // motor control speed
          tx_estring_value.append(Control_speed_arr[i]);
          
          tx_characteristic_string.writeValue(tx_estring_value.c_str());
          delay(10); // to make sure computer receive all data from BLE
        }
        Serial.print("Finish sending debugging data!");
        arr_index = 0; // for next run
        break;

      case SET_LINEAR_PID_GAIN: {
        // expects Kp|Ki|Kd --> "0.5|0.0|0.0"
        char gains_str[MAX_MSG_SIZE];
        success = robot_cmd.get_next_value(linear_Kp);
        if (!success) break;
        success = robot_cmd.get_next_value(linear_Ki);
        if (!success) break;
        success = robot_cmd.get_next_value(linear_Kd);
        if (!success) break;
        Serial.print("Set PID gains Kp = ");
        Serial.print(linear_Kp);
        Serial.print("|Ki = ");
        Serial.print(linear_Ki);
        Serial.print("|Kd = ");
        Serial.println(linear_Kd);
        break;
      }

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
  stop(); // start at a known stop state
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
      // Respond to any incoming BLE command
      if (rx_characteristic_string.written()) {
          handle_command();
      }
      if (linear_pid_running) {
        runLinearPIDController();
      }
    }
    stop();
    linear_pid_running = false;
    Serial.println("Disconnected from central");
    Serial.println("Hardstop, stopping motors");
  }
}







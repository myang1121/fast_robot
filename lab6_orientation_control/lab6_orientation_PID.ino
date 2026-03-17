// Lab 6 - orientation PID control
// this test program tests the car's ability to rotate to an angle (stationary orientation control), and when slightly kick, return to the angle
// control signal --> differential drive (rotateCW/ rotateCCW)
// PID input --> integrated gyroscope yaw

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

// int pwm_speed = 150; // PWM value 0-255
// int forward_1m_time = 1000;
// int backward_1m_time = 1000;
// int rotate_180_CCW_time = 1300;
// int rotate_360_CW_time = 2600;

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

////////////////////////////////////////////////////////// Lab 2 - IMU Sensor Reading //////////////////////////////////
#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include "math.h" // to use atan2 and M_PI

// I2C Macros
#define WIRE_PORT Wire // Your desired Wire port.      Used when "USE_SPI" is not defined
// The value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0
#define AD0_VAL 1

// ICM_20948_I2C object
ICM_20948_I2C myICM;

// variables for converting gyro_reading (angular change) to roll, pitch, yaw (angles)
float gyro_roll = 0.0;
float gyro_pitch = 0.0;
float gyro_yaw = 0.0;
unsigned long last_time = 0;
float dt = 0.0;

// variables for complementary filter
float accel_gain = 0.1; // initialized to 0.5, 50 percent weight on accel, 50 percent weight on gyro
float gyro_gain = 1. - accel_gain;
float compl_roll = 0.0;
float compl_pitch = 0.0;
float compl_yaw = 0.0; // equal gyro_yaw

////////////// IMU Functions ////////////////////////////////////
// to print data nicely
void printPaddedInt16b(int16_t val)
{
  if (val > 0)
  {
    Serial.print(" ");
    if (val < 10000)
    {
      Serial.print("0");
    }
    if (val < 1000)
    {
      Serial.print("0");
    }
    if (val < 100)
    {
      Serial.print("0");
    }
    if (val < 10)
    {
      Serial.print("0");
    }
  }
  else
  {
    Serial.print("-");
    if (abs(val) < 10000)
    {
      Serial.print("0");
    }
    if (abs(val) < 1000)
    {
      Serial.print("0");
    }
    if (abs(val) < 100)
    {
      Serial.print("0");
    }
    if (abs(val) < 10)
    {
      Serial.print("0");
    }
  }
  Serial.print(abs(val));
}

void printRawAGMT(ICM_20948_AGMT_t agmt)
{
  Serial.print("RAW. Acc [ ");
  printPaddedInt16b(agmt.acc.axes.x);
  Serial.print(", ");
  printPaddedInt16b(agmt.acc.axes.y);
  Serial.print(", ");
  printPaddedInt16b(agmt.acc.axes.z);
  Serial.print(" ], Gyr [ ");
  printPaddedInt16b(agmt.gyr.axes.x);
  Serial.print(", ");
  printPaddedInt16b(agmt.gyr.axes.y);
  Serial.print(", ");
  printPaddedInt16b(agmt.gyr.axes.z);
  Serial.print(" ], Mag [ ");
  printPaddedInt16b(agmt.mag.axes.x);
  Serial.print(", ");
  printPaddedInt16b(agmt.mag.axes.y);
  Serial.print(", ");
  printPaddedInt16b(agmt.mag.axes.z);
  Serial.print(" ], Tmp [ ");
  printPaddedInt16b(agmt.tmp.val);
  Serial.print(" ]");
  Serial.println();
}

void printFormattedFloat(float val, uint8_t leading, uint8_t decimals)
{
  float aval = abs(val);
  if (val < 0)
  {
    Serial.print("-");
  }
  else
  {
    Serial.print(" ");
  }
  for (uint8_t indi = 0; indi < leading; indi++)
  {
    uint32_t tenpow = 0;
    if (indi < (leading - 1))
    {
      tenpow = 1;
    }
    for (uint8_t c = 0; c < (leading - 1 - indi); c++)
    {
      tenpow *= 10;
    }
    if (aval < tenpow)
    {
      Serial.print("0");
    }
    else
    {
      break;
    }
  }
  if (val < 0)
  {
    Serial.print(-val, decimals);
  }
  else
  {
    Serial.print(val, decimals);
  }
}

void printScaledAGMT(ICM_20948_I2C *sensor)
{
  Serial.print("Scaled. Acc (mg) [ ");
  printFormattedFloat(sensor->accX(), 5, 2);
  Serial.print(", ");
  printFormattedFloat(sensor->accY(), 5, 2);
  Serial.print(", ");
  printFormattedFloat(sensor->accZ(), 5, 2);
  Serial.print(" ], Gyr (DPS) [ ");
  printFormattedFloat(sensor->gyrX(), 5, 2);
  Serial.print(", ");
  printFormattedFloat(sensor->gyrY(), 5, 2);
  Serial.print(", ");
  printFormattedFloat(sensor->gyrZ(), 5, 2);
  Serial.print(" ], Mag (uT) [ ");
  printFormattedFloat(sensor->magX(), 5, 2);
  Serial.print(", ");
  printFormattedFloat(sensor->magY(), 5, 2);
  Serial.print(", ");
  printFormattedFloat(sensor->magZ(), 5, 2);
  Serial.print(" ], Tmp (C) [ ");
  printFormattedFloat(sensor->temp(), 5, 2);
  Serial.print(" ]");
  Serial.println();
}
// to get accel, gyro, compl value
// convert accelerometer data into pitch and roll, update accel_roll and accel_pitch globals
void updateAccelPitchRoll() {
  // get a_x, a_y, a_z, accelerometer raw readings, but as floats
  float a_x = myICM.accX();
  float a_y = myICM.accY();
  float a_z = myICM.accZ();
  // calculate roll and pitch using atan2
  accel_roll = atan2(a_y, a_z) * 180.0 / M_PI; // atan2 returns radian --> convert to degrees using M_PI
  accel_pitch = atan2(a_x, a_z) * 180.0 / M_PI;
}

// Use a complementary filter to compute an estimate of pitch and roll that is both accurate (increase accel_gain reduce drift) and stable (increase gyro gain reduce vibration)
// gyro_gain = 1 - accel_gain
void updateGyroRollPitchYaw() {
  unsigned long now = millis();
  dt = (now - last_time) / 1000.0;  // ms -> seconds
  last_time = now;
  // calculate roll, pitch, and yaw (convert gyro_reading to angles in degrees)
  gyro_roll = gyro_roll + myICM.gyrX()*dt;
  gyro_pitch = gyro_pitch + myICM.gyrY()*dt;
  gyro_yaw = gyro_yaw + myICM.gyrZ()*dt; // no complementary data from accelerometer
}

void updateComplRollPitchYaw() {
  compl_roll = gyro_gain*(compl_roll + myICM.gyrX()*dt) + accel_gain*(accel_roll);
  compl_pitch = gyro_gain*(compl_pitch + myICM.gyrY()*dt) + accel_gain*(accel_pitch);
  compl_yaw = gyro_yaw;
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
#define SETPOINT 304 // 304mm = 1ft, expected distance
#define MIN_SPEED 0
#define MAX_SPEED 255
float Kp = 0.5;
float Ki = 0.0;
float Kd = 0.0;
int error = 0;
int sum_error = 0;
int previous_error = 0;
int derivative_error = 0;
int control_speed = 0;
bool pid_running = false;

// PID Controller w/ only proportional term (Kp)
void runPIDController() {
  // Continuously take a fresh ranging measurement
  // for liner PID, use sensor 1 only
  sensor1.startRanging();
  while (!sensor1.checkForDataReady()) {
      delay(1);
  }
  distance1 = sensor1.getDistance(); // updates global
  sensor1.clearInterrupt();
  sensor1.stopRanging();

  error = distance1 - SETPOINT;
  control_speed = (int)(Kp*abs(error)); // comment out when add I & D
  
  /* for I & D
  sum_error = sum_error + error;
  derivative_error = error - previous_error;
  control_speed = (int)(Kp*error + Ki*sum_error + Kd*derivative_error);
  previous_error = error;
  */

  control_speed = constrain(control_speed, MIN_SPEED, MAX_SPEED);

  ////////////// Collect data ////////////////////////////////////
  if (arr_index < ARRAY_SIZE) {
    T_arr[arr_index] = millis();
    Measured_distance_arr[arr_index] = distance1;
    Error_arr[arr_index] = error;
    Control_speed_arr[arr_index] = control_speed;
    arr_index++;
  }
  
  if (abs(error) < 20) { // comment out when add I & D
    stop();
  } else if (error > 0) {
    forward(control_speed);
  } else {
    backward(control_speed);
  }
}

////////////////////////////////////////////////////////// RobotCommand, communication b/w Artemis & computer //////////////////////////////////
RobotCommand robot_cmd(":|");
EString tx_estring_value;
// Commands
enum CommandTypes {
    START_PID,
    STOP_PID,
    SEND_PID_DATA,
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
      case START_PID:
        Serial.println("Start PID!");
        pid_running = true;
        break;

      case STOP_PID:
        Serial.println("Stop PID!");
        pid_running = false;
        stop(); // stop motors
        break;

      case SEND_PID_DATA:
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

  ////////////// IMU Sensor Setup /////////////////////////////
  // i2c setup
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
  //myICM.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial
  bool initialized = false;
  while (!initialized)
  {
    // begin i2c
    myICM.begin(WIRE_PORT, AD0_VAL);

    Serial.print(F("Initialization of the sensor returned: "));
    Serial.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      Serial.println("Trying again...");
      delay(500);
    }
    else
    {
      initialized = true;
    }
  }
  myICM.getAGMT();
  updateAccelPitchRoll();
  accel_roll_lpf  = accel_roll;
  accel_pitch_lpf = accel_pitch;

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
      if (pid_running) {
        runPIDController();
      }
      if (myICM.dataReady()) {
        myICM.getAGMT();         // The values are only updated when you call 'getAGMT'
        //printRawAGMT( myICM.agmt );     // Uncomment this to see the raw values, taken directly from the agmt structure
        printScaledAGMT(&myICM); // This function takes into account the scale settings from when the measurement was made to calculate the values with units
        updateAccelPitchRoll(); // get roll and pitch original signal from raw accelerometer reading (float a_x, a_y, a_z)
        updateGyroRollPitchYaw(); // get roll, pitch, yaw original signal from raw gyroscope reading (float g_x, g_y, g_z)
        updateComplRollPitchYaw();
      }
    }
    stop();
    pid_running = false;
    Serial.println("Disconnected from central");
    Serial.println("Hardstop, stopping motors");
  }
}
// Lab 6 - orientation PID control
// this test program tests the car's ability to rotate to an angle (stationary orientation control), and when slightly kick, return to the angle
// control signal --> differential drive (rotateCW/ rotateCCW)
// PID input --> integrated gyroscope yaw

////////////////////////////////////////////////////////// Libraries - all includes //////////////////////////////////
// ToF Sensor Libraries
#include <Wire.h>
#include "SparkFun_VL53L1X.h"
// IMU Sensor Libraries
#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include "math.h" // to use atan2 and M_PI
// BLE libraries
#include "BLECStringCharacteristic.h"
#include "EString.h"
#include "RobotCommand.h"
#include <ArduinoBLE.h>

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
// I2C Macros
#define WIRE_PORT Wire // Your desired Wire port.      Used when "USE_SPI" is not defined
// The value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0
#define AD0_VAL 1

// ICM_20948_I2C object
ICM_20948_I2C myICM;

// // variables for converting gyro_reading (angular change) to roll, pitch, yaw (angles)
// gyro yaw integration
float gyro_yaw = 0.0;
float gyro_z_bias = 0.0; // measured at rest, subtracted each step
unsigned long last_time = 0;
float dt = 0.0;

////////////// IMU Functions ////////////////////////////////////
// to get accel, gyro, compl value
// convert accelerometer data into pitch and roll, update accel_roll and accel_pitch globals
// read gyro, integrate yaw, update gyro_yaw global
void updateGyroYaw() {
  unsigned long now = millis();
  dt = (now - last_time)/1000.0; // ms -> seconds
  last_time = now;
  // subtract bias to reduce drift
  float gyro_z_corrected = myICM.gyrZ() - gyro_z_bias;
  gyro_yaw += gyro_z_corrected*dt;
}

// call this at startup with robot perfectly still for ~2s to measure bias
void calibrateGyroBias() {
  Serial.println("Calibrating gyro bias, keeping robot still...");
  float sum = 0.0;
  int samples = 200;
  for (int i = 0; i < samples; i++) {
    while (!myICM.dataReady()) {
      delay(1);
    }
    myICM.getAGMT();
    sum += myICM.gyrZ();
    delay(10);
  }
   gyro_z_bias = sum/samples;
  Serial.print("Gyro Z bias: ");
  Serial.println(gyro_z_bias);
}

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

////////////////////////////////////////////////////////// Lab 3 - ToF Sensor Reading //////////////////////////////////
// ToF Declarations & Variables
// One sensor objects
SFEVL53L1X sensor2; // side

// Declare distances as globals
// Measured distance
int distance2 = 0;

////////////////////////////////////////////////////////// Data Collection //////////////////////////////////
// For sending data over ble to python 
// Debugging data
// ToF
const int ARRAY_SIZE = 500;
int Measured_distance_arr[ARRAY_SIZE]; 
// IMU (for lab 6, orientation control)
float Yaw_arr[ARRAY_SIZE]; 
// Timestamp
unsigned long T_arr[ARRAY_SIZE]; 
// Array index
int arr_index = 0;

////////////////////////////////////////////////////////// Lab 1 - BLE //////////////////////////////////
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

////////////////////////////////////////////////////////// Lab 6 - Orientation PID //////////////////////////////////
#define MIN_SPEED 0
#define MAX_SPEED 255
// #define YAW_DEADZONE 2.0 // stop correcting if within +/- 2°
// widen the deadzone to trigger angle_read_already
#define YAW_DEADZONE 5.0
// settle delay, only log ToF and imu data after inside deadzone for a brief time to confirm it's actually stable
//unsigned long settled_since = 0;
#define SETTLE_MS 300 // angle rotation must settle for 300 ms before logging data

float orientation_setpoint = 0.0; // target yaw rotation angle set by ble
float orientation_Kp = 6.0;
float orientation_Ki = 0.0;
float orientation_Kd = 0.0;
float orientation_error = 0;
float orientation_sum_error = 0;
float orientation_control_speed = 0;
bool orientation_pid_running = false;

// Track whether already logged a reading at current angle
bool angle_read_already = false;

void runOrientationPIDController() {
  //Serial.println("PID LOOOOOOP running");
  // update yaw estimate from gyro
  if (myICM.dataReady()) {
    myICM.getAGMT();
    updateGyroYaw();
  } 
  // print statements for debugging only
  /*
  else {
    
    Serial.println("IMU not ready!");
  }
  */
  /*
  // print statements for debugging only
  Serial.print("gyro_yaw: ");
  Serial.print(gyro_yaw);
  Serial.print("| error: ");
  Serial.println(orientation_error);
  */

  orientation_error = orientation_setpoint - gyro_yaw;
  // P term only
  orientation_control_speed = orientation_Kp * abs(orientation_error);

  if (sensor2.checkForDataReady()) {
    distance2 = sensor2.getDistance();
    sensor2.clearInterrupt();
    sensor2.stopRanging();
    sensor2.startRanging();
  }

  /* w/ I & D
  orientation_sum_error += orientation_error;
  // Kd uses raw gyro directly (derivative of yaw = gyro reading)
  // this avoids derivative kick since not differencing error
  float gyro_z_corrected = myICM.gyrZ() - gyro_z_bias;
  orientation_control_speed = (int)(orientation_Kp*orientation_error + orientation_Ki*orientation_sum_error - orientation_Kd*gyro_z_corrected);
  */
  orientation_control_speed = constrain(orientation_control_speed, MIN_SPEED, MAX_SPEED);
  
  /*
  // add minimum speed floor so motor speed never drops below deadband even at small errors
  if (abs(orientation_error) >= YAW_DEADZONE) {
    // motor always gets at least 90 PWM when outside deadzone, so it moves rather than stall
    // for more consistent turns (not turn a lot, turn a little, turn a lot..., stall)
    orientation_control_speed = constrain(orientation_control_speed, 90, MAX_SPEED); // 90 is above deadband
  } else {
    orientation_control_speed = 0;
  }
  */

  ////////////// Collect data ////////////////////////////////////
  if (abs(orientation_error) < YAW_DEADZONE) {
    stop();
    /*
    if (settled_since == 0) {
      settled_since = millis(); // start settle timer
    }
    */
    // only log time after staying settled for SETTLE_MS
    //if (!angle_read_already && (millis() - settled_since > SETTLE_MS) && arr_index < ARRAY_SIZE) {
    if (!angle_read_already && arr_index < ARRAY_SIZE) {
      T_arr[arr_index]                 = millis();
      Yaw_arr[arr_index]               = gyro_yaw;
      Measured_distance_arr[arr_index] = distance2;
      arr_index++;
      angle_read_already = true;  // don't log again until target angle changes
      // for debugging
      Serial.print("Logged reading ");
      Serial.print(arr_index);          
    Serial.print(" yaw=");           
    Serial.println(gyro_yaw); 

    }
  } else {
    //settled_since = 0; // reset settle timer after resume moving (rotation to next setpoint angle)
    //angle_read_already = false;   // reset flag when moving toward new angle
    if (orientation_error > 0) {
      rotateCCW(orientation_control_speed);
    } else {
      rotateCW(orientation_control_speed);
    }
  }
}

////////////////////////////////////////////////////////// RobotCommand, communication b/w Artemis & computer //////////////////////////////////
RobotCommand robot_cmd(":|");
EString tx_estring_value;
// Commands
enum CommandTypes {
    START_MAPPING_360_ROTATION,
    STOP_MAPPING_360_ROTATION,
    SEND_MAPPING_DATA, 
    SET_ORIENTATION_SETPOINT, // update orientation_setpoint while running
    RESET_YAW, // zero out gyro_yaw, re-reference current angle, definding zero point at current boot up physical position
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
      case START_MAPPING_360_ROTATION:
        //settled_since = 0;
        Serial.println("Start mapping!");
        orientation_sum_error = 0;
        arr_index = 0;
        orientation_pid_running = true;
        angle_read_already = false;
        last_time = millis(); // reset dt reference
        sensor2.startRanging();
        break;

      case STOP_MAPPING_360_ROTATION:
        Serial.println("Stop Mapping!");
        orientation_pid_running = false; 
        stop(); // stop motors
        sensor2.stopRanging();
        break;
      
      case SEND_MAPPING_DATA:
        Serial.println("Sending orientation debugging data!");
        for (int i = 0; i < arr_index; i++) {
          tx_estring_value.clear();

          tx_estring_value.append("T:"); // timestamp
          tx_estring_value.append((int)T_arr[i]);
          tx_estring_value.append("|Y:"); // measured distance from ToF sensor reading
          tx_estring_value.append((float)Yaw_arr[i]);
          tx_estring_value.append("|D:"); // measured distance from ToF sensor reading
          tx_estring_value.append(Measured_distance_arr[i]);
          tx_characteristic_string.writeValue(tx_estring_value.c_str());
          delay(10); // to make sure computer receive all data from BLE
        }
        Serial.print("Finish sending debugging data!");
        arr_index = 0; // for next run
        break;
      case SET_ORIENTATION_SETPOINT: { // add curly bracket b/c define variable in case statement
        //settled_since = 0;
        float new_setpoint;
        success = robot_cmd.get_next_value(new_setpoint);
        if (success) {
          orientation_setpoint = new_setpoint;
          angle_read_already = false; // one reading each target angle
          Serial.print("New orientation setpoint: ");
          Serial.println(orientation_setpoint);
        }
        break;
      }
      case RESET_YAW:
        gyro_yaw = 0.0;
        orientation_setpoint = 0.0;
        orientation_sum_error = 0;
        angle_read_already = false;
        Serial.println("Yaw reset to 0");
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
  Serial.println("Lab 9 - Orientation PID");
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
  WIRE_PORT.begin(); // start I2C
  WIRE_PORT.setClock(400000);
  //myICM.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial
  bool initialized = false;
  while (!initialized)
  {
    // begin i2c
    myICM.begin(WIRE_PORT, AD0_VAL);

    Serial.print(F("Initialization of IMU sensor returned: "));
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
  Serial.println("IMU sensor ready!");
  
  calibrateGyroBias(); // robot still
  last_time = millis(); // initialize last_time 

  ////////////// 1 ToF Sensor Setup /////////////////////////////
  if (sensor2.begin() != 0) {
      Serial.println("Sensor 2 failed to begin. Please check wiring. Freezing...");
      while (1);
  }
  Serial.println("Sensor 2 OK!");
  sensor2.setDistanceModeLong();
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
      if (orientation_pid_running) {
        runOrientationPIDController();
      }
    }
    stop();
    orientation_pid_running = false;
    Serial.println("Disconnected from central");
    Serial.println("Hardstop, stopping motors");
  }
}

// Lab 12 - path planning and execution
// open loop forward drive (timed PWM) + PIC turning
// send DRIVE_OPEN_LOOP to translate
// send SET_ORIENTATION_SETPOINT to turn

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
#define MIN_TURN_PWM 80 // maybe not needed
#define SETTLE_MS 300 // angle rotation must settle for 300 ms before logging data

float orientation_setpoint = 0.0; // target yaw rotation angle set by ble
float orientation_Kp = 6.0;
float orientation_Ki = 0.0;
float orientation_Kd = 0.0;
float orientation_error = 0;
float orientation_sum_error = 0;
float orientation_control_speed = 0;
bool orientation_pid_running = false;

void runOrientationPIDController() {
  if (myICM.dataReady()) {
    myICM.getAGMT();
    updateGyroYaw();
  }

  orientation_error = orientation_setpoint - gyro_yaw;
  // wrap to (-180, 180] so the PID always takes the shortest-path turn
  while (orientation_error >   180.0) orientation_error -= 360.0;
  while (orientation_error <= -180.0) orientation_error += 360.0;

  orientation_control_speed = orientation_Kp * fabs(orientation_error);
  orientation_control_speed = constrain(orientation_control_speed, MIN_SPEED, MAX_SPEED);

  if (abs(orientation_error) < YAW_DEADZONE) {
    stop();
  } else {
    // friction floor: ensure motors actually move
    orientation_control_speed = max(orientation_control_speed, (float)MIN_TURN_PWM);
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
    RESET_YAW,
    START_PID,
    STOP_PID,
    SET_ORIENTATION_SETPOINT,
    DRIVE_OPEN_LOOP,
    SET_ORIENTATION_GAINS,
    GET_YAW,
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
      case RESET_YAW:
      gyro_yaw             = 0.0;
      orientation_setpoint = 0.0;
      orientation_sum_error = 0.0;
      Serial.println("[CMD] Yaw reset to 0");
      break;

    case START_PID:
      orientation_sum_error = 0.0;
      orientation_pid_running = true;
      last_time = millis();
      Serial.println("[CMD] PID started");
      break;

    case STOP_PID:
      orientation_pid_running = false;
      stop();
      Serial.println("[CMD] PID stopped");
      break;

    case SET_ORIENTATION_SETPOINT: {
      float new_setpoint;
      if (!robot_cmd.get_next_value(new_setpoint)) break;
      orientation_setpoint = new_setpoint;
      Serial.print("[CMD] New setpoint: ");
      Serial.println(orientation_setpoint);
      break;
    }

    case DRIVE_OPEN_LOOP: {
      int pwm_in, duration_ms;
      if (!robot_cmd.get_next_value(pwm_in))      break;
      if (!robot_cmd.get_next_value(duration_ms)) break;

      Serial.print("[CMD] Drive open-loop, pwm=");
      Serial.print(pwm_in);
      Serial.print(" duration_ms=");
      Serial.println(duration_ms);

      // Pause PID so it doesn't fight the forward drive
      bool was_pid_on = orientation_pid_running;
      orientation_pid_running = false;

      forward(pwm_in);
      unsigned long t0 = millis();
      while ((millis() - t0) < (unsigned long)duration_ms) {
        // keep integrating yaw during the drive so we know where we ended up
        if (myICM.dataReady()) {
          myICM.getAGMT();
          updateGyroYaw();
        }
        delay(2);
      }
      stop();

      // Snap setpoint to actual end-of-drive heading so PID doesn't kick
      // (handles the case where the robot drifted slightly during the drive)
      orientation_setpoint = gyro_yaw;
      orientation_pid_running = was_pid_on;

      Serial.print("[CMD] Drive done. yaw=");
      Serial.println(gyro_yaw);
      break;
    }

    case SET_ORIENTATION_GAINS: {
      float kp, ki, kd;
      if (!robot_cmd.get_next_value(kp)) break;
      if (!robot_cmd.get_next_value(ki)) break;
      if (!robot_cmd.get_next_value(kd)) break;
      orientation_Kp = kp;
      orientation_Ki = ki;
      orientation_Kd = kd;
      Serial.print("[CMD] Gains Kp=");
      Serial.print(kp);
      Serial.print(" Ki=");
      Serial.print(ki);
      Serial.print(" Kd=");
      Serial.println(kd);
      break;
    }

    case GET_YAW:
      // fresh yaw
      if (myICM.dataReady()) {
        myICM.getAGMT();
        updateGyroYaw();
      }
      tx_estring_value.clear();
      tx_estring_value.append("Y:");
      tx_estring_value.append(gyro_yaw);
      tx_characteristic_string.writeValue(tx_estring_value.c_str());
      Serial.print("[CMD] Sent yaw=");
      Serial.println(gyro_yaw);
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
  Serial.println("Lab 12 - path planning");
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
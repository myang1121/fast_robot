#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include "math.h" // to use atan2 and M_PI
//////////// Libraries for BLE ////////////
#include "BLECStringCharacteristic.h"
#include "EString.h"
#include "RobotCommand.h"
#include <ArduinoBLE.h>
//////////// BLE UUIDs ////////////
#define BLE_UUID_TEST_SERVICE "15bb5de7-5941-4ba2-bda0-784bb8817a1b"

#define BLE_UUID_RX_STRING "9750f60b-9c9c-4158-b620-02ec9521cd99"

#define BLE_UUID_TX_FLOAT "27616294-3063-4ecc-b60b-3470ddef2938"
#define BLE_UUID_TX_STRING "f235a225-6735-4d73-94cb-ee5dfce9ba83"

//////////// Serial Monitor & I2C Macros ////////////
#define SERIAL_PORT Serial

#define WIRE_PORT Wire // Your desired Wire port.      Used when "USE_SPI" is not defined
// The value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0
#define AD0_VAL 1

ICM_20948_I2C myICM; // Create an ICM_20948_I2C object

//////////// Accel, Gyro, Compl, Global Variables --> use ble to send to python ////////////
// variables for accelerometer roll (x) and pitch (y) 
float accel_roll = 0.0;
float accel_pitch = 0.0;
// variables for accelerometer roll (x) and pitch (y) low pass filtered
float accel_roll_lpf = 0.0;
float accel_pitch_lpf = 0.0;
// variables for accelerometer noise
const int ARRAY_SIZE = 100;
float accel_pitch_arr[ARRAY_SIZE];
float accel_roll_arr[ARRAY_SIZE];

// variables for converting gyro_reading (angular change) to roll, pitch, yaw (angles)
float gyro_roll = 0.0;
float gyro_pitch = 0.0;
float gyro_yaw = 0.0;
float dt = 0.001; // small dt...sampling frequency?

// variables for complementary filter
float accel_gain = 0.5; // initialized to 0.5, 50 percent weight on accel, 50 percent weight on gyro
float gyro_gain = 1. - accel_gain;
float compl_roll = 0.0;
float compl_pitch = 0.0;
float compl_yaw = 0.0; // equal gyro_yaw

//////////// BLE Global Variables & lab 1 global variables ////////////
BLEService testService(BLE_UUID_TEST_SERVICE);

BLECStringCharacteristic rx_characteristic_string(BLE_UUID_RX_STRING, BLEWrite, MAX_MSG_SIZE);

BLEFloatCharacteristic tx_characteristic_float(BLE_UUID_TX_FLOAT, BLERead | BLENotify);
BLECStringCharacteristic tx_characteristic_string(BLE_UUID_TX_STRING, BLERead | BLENotify, MAX_MSG_SIZE);

// for task 6
unsigned long arr[ARRAY_SIZE]; 

// for task 7
float temp_arr[ARRAY_SIZE]; // store temperature reading

// RX
RobotCommand robot_cmd(":|");

// TX
EString tx_estring_value;
float tx_float_value = 0.0;

long interval = 500;
static long previousMillis = 0;
unsigned long currentMillis = 0;

//////////// BLE case statements, declare command and handle command ////////////
enum CommandTypes
{
    PING,
    ECHO,
    GET_TIME_MILLIS, // task 3
    GET_CURRRENT_TIME_LOOP,
    STORE_TIME_STAMP, // task 6
    SEND_TIME_DATA,
    STORE_TIME_TEMP_STAMP, // modify later to get time_imu_stamp
    GET_TEMP_READINGS, // modify later to get time_imu_stamp

    // Accelerometer
    GET_ACCEL_DATA,
    GET_ACCEL_NOISE,
    // Gyroscope
    GET_GYRO_DATA,
    // Complementary Filtered Signal
    GET_COMPL_DATA,
};
void handle_command()
{   
    // Set the command string from the characteristic value
    robot_cmd.set_cmd_string(rx_characteristic_string.value(),
                             rx_characteristic_string.valueLength());

    bool success;
    int cmd_type = -1;

    // Get robot command type (an integer)
    /* NOTE: THIS SHOULD ALWAYS BE CALLED BEFORE get_next_value()
     * since it uses strtok internally (refer RobotCommand.h and 
     * https://www.cplusplus.com/reference/cstring/strtok/)
     */
    success = robot_cmd.get_command_type(cmd_type);

    // Check if the last tokenization was successful and return if failed
    if (!success) {
        return;
    }

    // Handle the command type accordingly
    switch (cmd_type) {
        /*
         * Write "PONG" on the GATT characteristic BLE_UUID_TX_STRING
         */
        case PING:
            tx_estring_value.clear();
            tx_estring_value.append("PONG");
            tx_characteristic_string.writeValue(tx_estring_value.c_str());

            Serial.print("Sent back: ");
            Serial.println(tx_estring_value.c_str());

            break;

        case ECHO:
            char char_arr[MAX_MSG_SIZE];

            // Extract the next value from the command string as a character array
            success = robot_cmd.get_next_value(char_arr);
            if (!success)
                return;
            
            tx_estring_value.clear();
            tx_estring_value.append("Robot says -> ");
            tx_estring_value.append(char_arr);
            tx_estring_value.append(" :)");
            tx_characteristic_string.writeValue(tx_estring_value.c_str());

            Serial.print("Sent back: ");
            Serial.println(tx_estring_value.c_str());
            
            break;

        /*
         * GET_TIME_MILLIS
         * Robot reply write a string such as “T:123456” to the string characteristic
         */
        
        case GET_TIME_MILLIS:
            unsigned long millisTime;
            millisTime = millis();
            tx_estring_value.clear();
            tx_estring_value.append("T:");
            tx_estring_value.append((int)millisTime);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());

            Serial.print("Sent back: ");
            Serial.println(tx_estring_value.c_str());
            
            break;

        /*
         * Task 5
         * GET_CURRENT_TIME_LOOP
         * Write a loop to get current time in miliseconds.
         * Send the current times to laptop to be received and processed by the notification handler.
         * Collect these values for a few seconds and use time stamps to determine how fast messages can be sent.
         */
        case GET_CURRRENT_TIME_LOOP:
            unsigned long millisTime_loop;
            for (int i = 0; i < 100; i++) {
                millisTime_loop = millis();
                tx_estring_value.clear();
                tx_estring_value.append("T:");
                tx_estring_value.append((int)millisTime_loop);
                // after notification handler starts notify, this loop continuously feed the notification handler data (current time) to be printed
                tx_characteristic_string.writeValue(tx_estring_value.c_str());
                // this is what the laptop (python jupyter notebook) sees
                Serial.println(tx_estring_value.c_str());
            }
            break;
        
        /*
         * Task 6
         * STORE_TIME_STAMP
         * Rather than send each time stamp, place each time stamp into the global array arr
         * Logic to determine when array is full so don't "over fill" the array
         */
        case STORE_TIME_STAMP:
            for (int i = 0; i < ARRAY_SIZE; i++) {
                // rather than writeValue, which sends the data to the laptop
                // store in array
                arr[i] = millis();
            }
            break;
        
        /*
         * SEND_TIME_DATA
         * Loops the array
         * Send each data point as a string to laptop to be processed
         */
        case SEND_TIME_DATA:
            for (int i = 0; i < ARRAY_SIZE; i++) {
                tx_estring_value.clear();
                tx_estring_value.append("T:");
                tx_estring_value.append((int)arr[i]);
                // send each data point to laptop
                tx_characteristic_string.writeValue(tx_estring_value.c_str());
                delay(10);
            }

            break;

        /*
         * Task 7
         * STORE_TIME_TEMP_STAMP
         * Element in time stamp and temperature reading array should correspond
         * Send each data point as a string to laptop to be processed
         */
        case STORE_TIME_TEMP_STAMP:
            for (int i = 0; i < ARRAY_SIZE; i++) {
                arr[i] = millis();
                temp_arr[i] = getTempDegF();
            }
            break;

        /*
         * GET_TEMP_READINGS
         * Send time and temp reading to laptop
         */
        case GET_TEMP_READINGS:
            for (int i = 0; i < ARRAY_SIZE; i++) {
                tx_estring_value.clear();
                tx_estring_value.append("Temperature:");
                tx_estring_value.append(temp_arr[i]);
                tx_estring_value.append(",Time:");
                tx_estring_value.append((int)arr[i]);
                // send each data point to laptop
                tx_characteristic_string.writeValue(tx_estring_value.c_str());
                delay(10);
            }
        break;

        // Accelerometer
        case GET_ACCEL_DATA:
          updateAccelPitchRoll();
          // lpf --> angle_lpf = alpha*angle_no_lpf + (1-alpha)*angle_lpf_prev
          accel_roll_lpf = accel_gain*accel_roll+ gyro_gain*accel_roll_lpf;
          accel_pitch_lpf = accel_gain*accel_pitch+ gyro_gain*accel_pitch_lpf;

          // through ble, send to python
          tx_estring_value.clear();
          tx_estring_value.append("Accel_pitch:");
          tx_estring_value.append(accel_pitch);
          tx_estring_value.append(",Accel_roll:");
          tx_estring_value.append(accel_roll);
          tx_estring_value.append(",Accel_roll_lpf:");
          tx_estring_value.append(accel_roll_lpf);
          tx_estring_value.append(",Accel_pitch_lpf:");
          tx_estring_value.append(accel_pitch_lpf);
          tx_estring_value.append(",T:");
          tx_estring_value.append((int)millis());
          tx_characteristic_string.writeValue(tx_estring_value.c_str());

          break;

        case GET_ACCEL_NOISE:
        // store in array
          for (int i = 0; i < ARRAY_SIZE; i ++) {
            if (myICM.dataReady()) { // for sampling at 200Hz
              myICM.getAGMT();
              updateAccelPitchRoll();
              accel_pitch_arr[i] = accel_pitch;
              accel_roll_arr[i] = accel_roll;
              arr[i] = millis();
            }
            delay(5); // 5ms = 200Hz --> record as fast and consistently as possible
          }
          // through ble, send to python
          for (int i = 0; i < ARRAY_SIZE; i ++) {
            tx_estring_value.clear();
            tx_estring_value.append("Accel_pitch:");
            tx_estring_value.append(accel_pitch_arr[i]);
            tx_estring_value.append(",Accel_roll:");
            tx_estring_value.append(accel_roll_arr[i]);
            tx_estring_value.append(",T:");
            tx_estring_value.append((int)arr[i]);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
            delay(10); // to ensure all 100 samples send to python
          } 
          break;

        // Gyroscope
        case GET_GYRO_DATA:
          updateGyroRollPitchYaw();
          // through ble, send to python
          tx_estring_value.clear();
          tx_estring_value.append("Gyro_pitch:");
          tx_estring_value.append(gyro_pitch);
          tx_estring_value.append(",Gyro_roll:");
          tx_estring_value.append(gyro_roll);
          tx_estring_value.append(",Gyro_yaw:");
          tx_estring_value.append(gyro_yaw);
          tx_estring_value.append(",T:");
          tx_estring_value.append((int)millis());
          tx_characteristic_string.writeValue(tx_estring_value.c_str());
          
          break;

        // Complementary Filtered Signal
        case GET_COMPL_DATA:
          compl_roll = gyro_gain*(compl_roll + myICM.gyrX()*dt) + accel_gain*(accel_roll);
          compl_pitch = gyro_gain*(compl_pitch + myICM.gyrY()*dt) + accel_gain*(accel_pitch);
          compl_yaw = gyro_yaw;
          // through ble, send to python
          tx_estring_value.clear();
          tx_estring_value.append("Compl_pitch:");
          tx_estring_value.append(compl_pitch);
          tx_estring_value.append(",Compl_roll:");
          tx_estring_value.append(compl_roll);
          tx_estring_value.append(",Compl_yaw:");
          tx_estring_value.append(compl_yaw);
          tx_estring_value.append(",T:");
          tx_estring_value.append((int)millis());
          tx_characteristic_string.writeValue(tx_estring_value.c_str());
          break;

        /* 
         * The default case may not capture all types of invalid commands.
         * It is safer to validate the command string on the central device (in python)
         * before writing to the characteristic.
         */
        default:
            Serial.print("Invalid Command Type: ");
            Serial.println(cmd_type);
            break;
    }
}

//////////// Start! ////////////
void setup()
{
  SERIAL_PORT.begin(115200);

  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  // blink LED 3 times slowly on start-up
  digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
  delay(2000);                      // wait for a second
  digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
  delay(2000);                      // wait for a second
  digitalWrite(LED_BUILTIN, HIGH);
  delay(2000);    
  digitalWrite(LED_BUILTIN, LOW);
  delay(2000);    
  digitalWrite(LED_BUILTIN, HIGH);
  delay(2000);    
  digitalWrite(LED_BUILTIN, LOW);
  delay(2000);    
  
  // i2c setup
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);

  //myICM.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

  bool initialized = false;
  while (!initialized)
  {
    // begin i2c
    myICM.begin(WIRE_PORT, AD0_VAL);

    SERIAL_PORT.print(F("Initialization of the sensor returned: "));
    SERIAL_PORT.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      SERIAL_PORT.println("Trying again...");
      delay(500);
    }
    else
    {
      initialized = true;
    }
  }

  //////////// BLE Setup ////////////
  BLE.begin();
  // Set advertised local name and service
    BLE.setDeviceName("Artemis BLE");
    BLE.setLocalName("Artemis BLE");
    BLE.setAdvertisedService(testService);

    // Add BLE characteristics
    testService.addCharacteristic(tx_characteristic_float);
    testService.addCharacteristic(tx_characteristic_string);
    testService.addCharacteristic(rx_characteristic_string);

    // Add BLE service
    BLE.addService(testService);

    // Initial values for characteristics
    // Set initial values to prevent errors when reading for the first time on central devices
    tx_characteristic_float.writeValue(0.0);

    /*
     * An example using the EString
     */
    // Clear the contents of the EString before using it
    tx_estring_value.clear();

    // Append the string literal "[->"
    tx_estring_value.append("[->");

    // Append the float value
    tx_estring_value.append(9.0);

    // Append the string literal "<-]"
    tx_estring_value.append("<-]");

    // Write the value to the characteristic
    tx_characteristic_string.writeValue(tx_estring_value.c_str());

    // Output MAC Address
    Serial.print("Advertising BLE with MAC: ");
    Serial.println(BLE.address());

    BLE.advertise();
}

//////////// Main loop! ////////////
void loop()
{
  // Listen for connections
  BLEDevice central = BLE.central();

  // If a central is connected to the peripheral
  // print MAC address
  if (central) {
      Serial.print("Connected to: ");
      Serial.println(central.address());

      // While central is connected
      while (central.connected()) {
          // Send data
          write_data();
          // Read data
          read_data();
      }
      Serial.println("Disconnected");
  }
  //////////// IMU ////////////
  if (myICM.dataReady())
  {
    myICM.getAGMT();         // The values are only updated when you call 'getAGMT'
    //printRawAGMT( myICM.agmt );     // Uncomment this to see the raw values, taken directly from the agmt structure
    printScaledAGMT(&myICM); // This function takes into account the scale settings from when the measurement was made to calculate the values with units
    updateAccelPitchRoll(); // get roll and pitch original signal from raw accelerometer reading (float a_x, a_y, a_z)
    updateGyroRollPitchYaw(); // get roll, pitch, yaw original signal from raw gyroscope reading (float g_x, g_y, g_z)
    // print accel roll and pitch
    SERIAL_PORT.print("Accel Roll (°) = ");
    SERIAL_PORT.println(accel_roll);
    SERIAL_PORT.print("Accel Pitch (°)= ");
    SERIAL_PORT.println(accel_pitch);
    // print gyro roll, pitch, yaw
    SERIAL_PORT.print("Gyro Roll (°) = ");
    SERIAL_PORT.println(gyro_roll);
    SERIAL_PORT.print("Gyro Pitch (°)= ");
    SERIAL_PORT.println(gyro_pitch);
    SERIAL_PORT.print("Gyro Yaw (°)= ");
    SERIAL_PORT.println(gyro_yaw);
    delay(300);
  }
  else
  {
    SERIAL_PORT.println("Waiting for data");
    delay(500);
  }

}

//////////// helper functions to print the data nicely ////////////
void printPaddedInt16b(int16_t val)
{
  if (val > 0)
  {
    SERIAL_PORT.print(" ");
    if (val < 10000)
    {
      SERIAL_PORT.print("0");
    }
    if (val < 1000)
    {
      SERIAL_PORT.print("0");
    }
    if (val < 100)
    {
      SERIAL_PORT.print("0");
    }
    if (val < 10)
    {
      SERIAL_PORT.print("0");
    }
  }
  else
  {
    SERIAL_PORT.print("-");
    if (abs(val) < 10000)
    {
      SERIAL_PORT.print("0");
    }
    if (abs(val) < 1000)
    {
      SERIAL_PORT.print("0");
    }
    if (abs(val) < 100)
    {
      SERIAL_PORT.print("0");
    }
    if (abs(val) < 10)
    {
      SERIAL_PORT.print("0");
    }
  }
  SERIAL_PORT.print(abs(val));
}

void printRawAGMT(ICM_20948_AGMT_t agmt)
{
  SERIAL_PORT.print("RAW. Acc [ ");
  printPaddedInt16b(agmt.acc.axes.x);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.acc.axes.y);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.acc.axes.z);
  SERIAL_PORT.print(" ], Gyr [ ");
  printPaddedInt16b(agmt.gyr.axes.x);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.gyr.axes.y);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.gyr.axes.z);
  SERIAL_PORT.print(" ], Mag [ ");
  printPaddedInt16b(agmt.mag.axes.x);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.mag.axes.y);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.mag.axes.z);
  SERIAL_PORT.print(" ], Tmp [ ");
  printPaddedInt16b(agmt.tmp.val);
  SERIAL_PORT.print(" ]");
  SERIAL_PORT.println();
}

void printFormattedFloat(float val, uint8_t leading, uint8_t decimals)
{
  float aval = abs(val);
  if (val < 0)
  {
    SERIAL_PORT.print("-");
  }
  else
  {
    SERIAL_PORT.print(" ");
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
      SERIAL_PORT.print("0");
    }
    else
    {
      break;
    }
  }
  if (val < 0)
  {
    SERIAL_PORT.print(-val, decimals);
  }
  else
  {
    SERIAL_PORT.print(val, decimals);
  }
}

void printScaledAGMT(ICM_20948_I2C *sensor)
{
  SERIAL_PORT.print("Scaled. Acc (mg) [ ");
  printFormattedFloat(sensor->accX(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->accY(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->accZ(), 5, 2);
  SERIAL_PORT.print(" ], Gyr (DPS) [ ");
  printFormattedFloat(sensor->gyrX(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->gyrY(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->gyrZ(), 5, 2);
  SERIAL_PORT.print(" ], Mag (uT) [ ");
  printFormattedFloat(sensor->magX(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->magY(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->magZ(), 5, 2);
  SERIAL_PORT.print(" ], Tmp (C) [ ");
  printFormattedFloat(sensor->temp(), 5, 2);
  SERIAL_PORT.print(" ]");
  SERIAL_PORT.println();
}

//////////// helper functions to get accel, gyro, compl value ////////////
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
  // calculate roll, pitch, and yaw (convert gyro_reading to angles in degrees)
  gyro_roll = gyro_roll + myICM.gyrX()*dt;
  gyro_pitch = gyro_pitch + myICM.gyrY()*dt;
  gyro_yaw = gyro_yaw + myICM.gyrZ()*dt; // no complementary data from accelerometer
}

//////////// helper function for ble ////////////

void write_data()
{
    currentMillis = millis();
    if (currentMillis - previousMillis > interval) {

        tx_float_value = tx_float_value + 0.5;
        tx_characteristic_float.writeValue(tx_float_value);

        if (tx_float_value > 10000) {
            tx_float_value = 0;
            
        }

        previousMillis = currentMillis;
    }
}

void read_data()
{
    // Query if the characteristic value has been written by another BLE device
    if (rx_characteristic_string.written()) {
        handle_command();
    }
}


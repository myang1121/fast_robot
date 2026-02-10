/****************************************************************
 * Example1_Basics.ino
 * ICM 20948 Arduino Library Demo
 * Use the default configuration to stream 9-axis IMU data
 * Owen Lyke @ SparkFun Electronics
 * Original Creation Date: April 17 2019
 *
 * Please see License.md for the license information.
 *
 * Distributed as-is; no warranty is given.
 ***************************************************************/
#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include "math.h" // to use atan2 and M_PI
// connection to python
#include "BLECStringCharacteristic.h"
#include "EString.h"
#include "RobotCommand.h"
#include <ArduinoBLE.h>

//////////// BLE UUIDs ////////////
#define BLE_UUID_TEST_SERVICE "15bb5de7-5941-4ba2-bda0-784bb8817a1b"
#define BLE_UUID_RX_STRING "9750f60b-9c9c-4158-b620-02ec9521cd99"
#define BLE_UUID_TX_FLOAT "27616294-3063-4ecc-b60b-3470ddef2938"
#define BLE_UUID_TX_STRING "f235a225-6735-4d73-94cb-ee5dfce9ba83"
//////////// BLE UUIDs ////////////

//////////// Global Variables ////////////
BLEService testService(BLE_UUID_TEST_SERVICE);
BLECStringCharacteristic rx_characteristic_string(BLE_UUID_RX_STRING, BLEWrite, MAX_MSG_SIZE);
BLEFloatCharacteristic tx_characteristic_float(BLE_UUID_TX_FLOAT, BLERead | BLENotify);
BLECStringCharacteristic tx_characteristic_string(BLE_UUID_TX_STRING, BLERead | BLENotify, MAX_MSG_SIZE);

// RX
RobotCommand robot_cmd(":|");

// TX
EString tx_estring_value;
float tx_float_value = 0.0;

long interval = 500;
static long previousMillis = 0;
unsigned long currentMillis = 0;

//#define USE_SPI       // Uncomment this to use SPI

#define SERIAL_PORT Serial

#define SPI_PORT SPI // Your desired SPI port.       Used only when "USE_SPI" is defined
#define CS_PIN 2     // Which pin you connect CS to. Used only when "USE_SPI" is defined

#define WIRE_PORT Wire // Your desired Wire port.      Used when "USE_SPI" is not defined
// The value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0
#define AD0_VAL 1

#ifdef USE_SPI
ICM_20948_SPI myICM; // If using SPI create an ICM_20948_SPI object
#else
ICM_20948_I2C myICM; // Otherwise create an ICM_20948_I2C object
#endif

// variables for accelerometer roll (x) and pitch (y)
float accel_roll = 0.0;
float accel_pitch = 0.0;

// variables for converting accel_reading to roll, pitch, yaw using atan2, and for low pass filtered roll, pitch, yaw
float roll_low_passed_new;

// variables for converting gyro_reading (angular change) to roll, pitch, yaw (angles)
float gyro_roll_new;
float dt = 0.001; // small dt...sampling frequency?

// accelerometer (alpha) and gyro gain in float for complementary filter
float accel_gain = 0.5; // initialized to 0.5, 50 percent weight on accel, 50 percent weight on gyro
float gyro_gain = 1. - accel_gain;

// variables for complementary filter
float compl_roll_new;
float compl_pitch_new;
float compl_yaw_new;

//////////// Command Types ////////////
enum CommandTypes {
    PING,
    SEND_TWO_INTS,
    SEND_THREE_FLOATS,
    ECHO,
    DANCE,
    SET_VEL,
    GET_TIME_MILLIS,
    GET_IMU_DATA, // send pitch and roll value over ble to python 
};

void setup()
{
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

  SERIAL_PORT.begin(115200);
  while (!SERIAL_PORT)
  {
  };

#ifdef USE_SPI
  SPI_PORT.begin();
#else
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
#endif

  //myICM.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

  bool initialized = false;
  while (!initialized)
  {

#ifdef USE_SPI
    myICM.begin(CS_PIN, SPI_PORT);
#else
    myICM.begin(WIRE_PORT, AD0_VAL);
#endif

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
  tx_estring_value.append("[->"); 
  tx_estring_value.append(9.0); 
  tx_estring_value.append("<-]");
  tx_characteristic_string.writeValue(tx_estring_value.c_str());

  Serial.print("Advertising BLE with MAC: "); 
  Serial.println(BLE.address());
  BLE.advertise();
}

void loop()
{
  if (myICM.dataReady())
  {
    myICM.getAGMT();         // The values are only updated when you call 'getAGMT'
    //printRawAGMT( myICM.agmt );     // Uncomment this to see the raw values, taken directly from the agmt structure
    printScaledAGMT(&myICM); // This function takes into account the scale settings from when the measurement was made to calculate the values with units
    printAccelToPitchRoll(&myICM); // print roll and pitch original signal from raw accelerometer reading (float a_x, a_y, a_z)
    //PrintLowpassFilteredSignal(&myICM); // print roll and pitch low pass filtered signal
    //printGyroToPitchRollYaw(&myICM); // print roll, pitch, yaw original signal from raw gyroscope reading (float g_x, g_y, g_z)
    delay(300);
  }
  else
  {
    SERIAL_PORT.println("Waiting for data");
    delay(500);
  }
}

// Below here are some helper functions to print the data nicely!

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

// convert accelerometer data into pitch and roll
void printAccelToPitchRoll(ICM_20948_I2C *sensor) {
  // get a_x, a_y, a_z, accelerometer raw readings, but as floats
  float a_x = sensor->accX();
  float a_y = sensor->accY();
  float a_z = sensor->accZ();
  // calculate roll and pitch using atan2
  float roll = atan2(a_y, a_z) * 180.0 / M_PI; // atan2 returns radian --> convert to degrees
  float pitch = atan2(a_x, a_z) * 180.0 / M_PI;
  // print roll and pitch
  SERIAL_PORT.print("Roll (°) = ");
  SERIAL_PORT.println(roll);
  SERIAL_PORT.print("Pitch (°)= ");
  SERIAL_PORT.println(pitch);
}

// // simple lowpass filter on accelerometer data
// void PrintLowpassFilteredSignal(ICM_20948_I2C *sensor) {
//   // maybe i should call printAccelToPitchRow? instead of having it print, have it return soem value and let other helper function print the returned original atan2 calculated roll and pitch? 
//   float roll_low_passed_new = accel_gain*roll_raw + (1 - accel_gain)*roll_low_passed;
//   roll_low_passed = roll_low_passed_new;
// }


// void printGyroToPitchRollYaw(ICM_20948_I2C *sensor) {
//   // get g_x, g_y, g_z, gyroscope raw readings (rate of angular change in degrees/sec), but as floats
//   float g_x = sensor->gyrX();
//   float g_y = sensor->gyrY();
//   float g_z = sensor->gyrZ();
//   // calculate roll, pitch, and yaw (convert gyro_reading to angles in degrees)
//   float gyro_roll_new = gyro_roll + g_x*dt;
//   float gyro_pitch_new = gyro_pitch + g_y*dt;
//   float gyro_yaw_new = gyro_yaw + g_z*dt;
// }

// // Use a complementary filter to compute an estimate of pitch and roll that is both accurate (increase accel_gain reduce drift) and stable (increase gyro gain reduce vibration)
// // gyro_gain = 1 - accel_gain
// void printComplementaryFilteredSignal(ICM_20948_I2C *sensor) {
//   float compl_roll_new = (compl_roll + roll_g)(gyro_gain) + roll_a*accel_gain;
//   float compl_pitch_new = (compl_pitch + pitch_g)(gyro_gain) + pitch_a*accel_gain;
//   // no complementary data from accelerometer, yaw_a = raw accel reading, a_z
//   float compl_yaw_new = (compl_yaw + yaw_g)(gyro_gain) + yaw_a*accel_gain;

// }


#ifdef USE_SPI
void printScaledAGMT(ICM_20948_SPI *sensor)
{
#else
void printScaledAGMT(ICM_20948_I2C *sensor)
{
#endif
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

/******************************************************************************
I2C_ReadAllData.ino
BME280 Arduino and Teensy example
Marshall Taylor @ SparkFun Electronics
May 20, 2015
https://github.com/sparkfun/SparkFun_BME280_Arduino_Library

This sketch configures the BME280 to read all measurements.  The sketch also
displays the BME280's physical memory and what the driver perceives the
calibration words to be.

Resources:
Uses Wire.h for I2C operation
Uses SPI.h for SPI operation

Development environment specifics:
Arduino IDE 1.6.4
Teensy loader 1.23

This code is released under the [MIT License](http://opensource.org/licenses/MIT).
Please review the LICENSE.md file included with this example. If you have any questions
or concerns with licensing, please contact techsupport@sparkfun.com.
Distributed as-is; no warranty is given.
******************************************************************************/

/*
 * Wiring is as follows:
 *
 * ESP8266 DevBoard <-> BME280
 * 3.3v       <->       VIN
 * GND        <->       GND
 * D1         <->       SCL
 * D2         <->       SDA
 *
 * I2C address is 0x76 (not 0x77, which is the default)
 */

#define DEBUG_REST
#define DEBUG_ESP_SSL

#include <Arduino.h>
#include <stdint.h>

// JSON generator

#include <ArduinoJson.h>

// BM[E,P]280

#include "SparkFunBME280.h"
//Library allows either I2C or SPI, so include both.
#include "Wire.h"
#include "SPI.h"

// MPU9250

#include "quaternionFilters.h"
#include "MPU9250.h"

// WI-FI/WifiManager/REST

#include <ESP8266WiFi.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>
#include <RestClient.h>

typedef Palatis::RestClient<WiFiClientSecure, 443> RestClient_T;

const char* ssid     = "TP-LINK_4379";
const char* password = "TP-LINK_4379";

const char* host = "things.ubidots.com";
const char *uri = "/api/v1.6/devices/cngconnectedcooler";
#define UBIDOTS_TOKEN "UL7wpuTgnlsn04DwPFmytrr3cX5kDq"

// Use WiFiClientSecure class to create TCP connections

WiFiClientSecure client;
RestClient_T rest = RestClient_T(client, host);

// Global BM[E,P]280 object

BME280 mySensor;
bool printBME280 = true;

// Global MPU9250 object

MPU9250 myIMU;
bool printMPU9250 = true;
float xAxisRestValue = 0.0;
#define DOOR_CLOSED_VARIATION (400)	// If we're greater than +/- 700 of the x-axis value, consider the door open.
#define X_AXIS_MAG_BIAS (+470.0)
#define Y_AXIS_MAG_BIAS (+120.0)
#define Z_AXIS_MAG_BIAS (+125.0)

// Global cooler variables

enum DOORSTATE {
  doorOpen = 0,
  doorClosed = 1
} oldDoorState = doorClosed;

long doorOpenCount = 0;
long doorCloseCount = 0;

/*
 * Here is the JSON data format:
 *
 *

   {
     "deviceId": "COOLER_7",
     "messageTimeStamp": "1467370000000",
     "firmwareVersion": "ZVER1",
     "temperature": 12,
     "doorOpenCount": 11,
     "doorCloseCount": 11,
     "doorOpenTime": 33,
     "mobileNetworkId": "ZNTW1",
     "mobileCellularId": "ZCELL1",
     "latitude": 37.400794,
     "longitude": -122.109797,
     "mobileCellularType": "3G",
     "mobileRSSI": -70,
     "powerState": 0,
     "coolerState": 3,
     "wifiVisibleCount": 2
   }

*
* For an explanation, see https://github.wdf.sap.corp/IoT-SCB-India/Connected-X-Wiki/blob/master/restservices/devicedata.md
*
*
*/

// doorState (0 = OPEN, 1 = CLOSED)
// We assume that the door is closed on boot.

void postToCnG(float temperature, enum DOORSTATE currentDoorState) {

  // Create the body of the POST in JSON format.

  DynamicJsonBuffer jsonBuffer;

  JsonObject& root = jsonBuffer.createObject();

  root["temperature"] = temperature;

#if 0
  if (currentDoorState != oldDoorState) {
    if (currentDoorState == doorOpen) {
      doorOpenCount++;
      root["doorOpenCount"] = doorOpenCount;
    } else {
      doorCloseCount++;
      root["doorCloseCount"] = doorCloseCount;
    }
  }
#endif // 0

  int bufferLength = root.measureLength() + 1;

  char *body = (char *)malloc(bufferLength * sizeof(char));

  root.printTo(body, bufferLength);

  Serial.print("postToCnG: POSTing body: "); Serial.println(body);
  Serial.print("postToCnG: POSTing to: "); Serial.print("https://"); Serial.print(host); Serial.println(uri);
  Serial.println();

  rest.addHeader("X-Auth-Token", UBIDOTS_TOKEN);
  rest.setContentType("application/json");

  String response = rest.post(uri, body);

  Serial.print("postToCnG: POST returned "); Serial.println(response);
  Serial.println();

  free(body);
}

void setup()
{

  Serial.begin(115200);
  delay(5000);

  WiFiManager wifiManager;
  wifiManager.autoConnect(ssid, password);

  Serial.println();
  Serial.println("WiFi connected");
  Serial.print("IP address: "); Serial.println(WiFi.localIP());

  postToCnG(79.99, doorOpen);

  delay(10000000);

	// Setup for BM[E,P]280

	//***Driver settings********************************//
	//commInterface can be I2C_MODE or SPI_MODE
	//specify chipSelectPin using arduino pin names
	//specify I2C address.  Can be 0x77(default) or 0x76

	//For I2C, enable the following and disable the SPI section
	mySensor.settings.commInterface = I2C_MODE;
	mySensor.settings.I2CAddress = 0x76;

	//For SPI enable the following and dissable the I2C section
	//mySensor.settings.commInterface = SPI_MODE;
	//mySensor.settings.chipSelectPin = 10;


	//***Operation settings*****************************//

	//renMode can be:
	//  0, Sleep mode
	//  1 or 2, Forced mode
	//  3, Normal mode
	mySensor.settings.runMode = 3; //Normal mode

	//tStandby can be:
	//  0, 0.5ms
	//  1, 62.5ms
	//  2, 125ms
	//  3, 250ms
	//  4, 500ms
	//  5, 1000ms
	//  6, 10ms
	//  7, 20ms
	mySensor.settings.tStandby = 0;

	//filter can be off or number of FIR coefficients to use:
	//  0, filter off
	//  1, coefficients = 2
	//  2, coefficients = 4
	//  3, coefficients = 8
	//  4, coefficients = 16
	mySensor.settings.filter = 0;

	//tempOverSample can be:
	//  0, skipped
	//  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
	mySensor.settings.tempOverSample = 1;

	//pressOverSample can be:
	//  0, skipped
	//  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
    mySensor.settings.pressOverSample = 1;

	//humidOverSample can be:
	//  0, skipped
	//  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
	mySensor.settings.humidOverSample = 1;

	Serial.begin(115200);
	Serial.print("Program Started\n");
	Serial.print("Starting BME280... result of .begin(): 0x");

	//Calling .begin() causes the settings to be loaded
	delay(10);  //Make sure sensor had enough time to turn on. BME280 requires 2ms to start up.
	Serial.println(mySensor.begin(), HEX);

	Serial.print("Displaying ID, reset and ctrl regs\n");

	Serial.print("ID(0xD0): 0x");
	Serial.println(mySensor.readRegister(BME280_CHIP_ID_REG), HEX);
	Serial.print("Reset register(0xE0): 0x");
	Serial.println(mySensor.readRegister(BME280_RST_REG), HEX);
	Serial.print("ctrl_meas(0xF4): 0x");
	Serial.println(mySensor.readRegister(BME280_CTRL_MEAS_REG), HEX);
	Serial.print("ctrl_hum(0xF2): 0x");
	Serial.println(mySensor.readRegister(BME280_CTRL_HUMIDITY_REG), HEX);

	Serial.print("\n\n");

	Serial.print("Displaying all regs\n");
	uint8_t memCounter = 0x80;
	uint8_t tempReadData;
	for(int rowi = 8; rowi < 16; rowi++ )
	{
		Serial.print("0x");
		Serial.print(rowi, HEX);
		Serial.print("0:");
		for(int coli = 0; coli < 16; coli++ )
		{
			tempReadData = mySensor.readRegister(memCounter);
			Serial.print((tempReadData >> 4) & 0x0F, HEX);//Print first hex nibble
			Serial.print(tempReadData & 0x0F, HEX);//Print second hex nibble
			Serial.print(" ");
			memCounter++;
		}
		Serial.print("\n");
	}


	Serial.print("\n\n");

	Serial.print("Displaying concatenated calibration words\n");
	Serial.print("dig_T1, uint16: ");
	Serial.println(mySensor.calibration.dig_T1);
	Serial.print("dig_T2, int16: ");
	Serial.println(mySensor.calibration.dig_T2);
	Serial.print("dig_T3, int16: ");
	Serial.println(mySensor.calibration.dig_T3);

	Serial.print("dig_P1, uint16: ");
	Serial.println(mySensor.calibration.dig_P1);
	Serial.print("dig_P2, int16: ");
	Serial.println(mySensor.calibration.dig_P2);
	Serial.print("dig_P3, int16: ");
	Serial.println(mySensor.calibration.dig_P3);
	Serial.print("dig_P4, int16: ");
	Serial.println(mySensor.calibration.dig_P4);
	Serial.print("dig_P5, int16: ");
	Serial.println(mySensor.calibration.dig_P5);
	Serial.print("dig_P6, int16: ");
	Serial.println(mySensor.calibration.dig_P6);
	Serial.print("dig_P7, int16: ");
	Serial.println(mySensor.calibration.dig_P7);
	Serial.print("dig_P8, int16: ");
	Serial.println(mySensor.calibration.dig_P8);
	Serial.print("dig_P9, int16: ");
	Serial.println(mySensor.calibration.dig_P9);

	Serial.print("dig_H1, uint8: ");
	Serial.println(mySensor.calibration.dig_H1);
	Serial.print("dig_H2, int16: ");
	Serial.println(mySensor.calibration.dig_H2);
	Serial.print("dig_H3, uint8: ");
	Serial.println(mySensor.calibration.dig_H3);
	Serial.print("dig_H4, int16: ");
	Serial.println(mySensor.calibration.dig_H4);
	Serial.print("dig_H5, int16: ");
	Serial.println(mySensor.calibration.dig_H5);
	Serial.print("dig_H6, uint8: ");
	Serial.println(mySensor.calibration.dig_H6);

	Serial.println();

	// Setup for MPU9250

	// Read the WHO_AM_I register, this is a good test of communication
  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX);
  Serial.print(" I should be "); Serial.println(0x71, HEX);

	if (c == 0x71) // WHO_AM_I should always be 0x68
  {
    Serial.println("MPU9250 is online...");

    // Start by performing self test and reporting values
    myIMU.MPU9250SelfTest(myIMU.SelfTest);
    Serial.print("x-axis self test: acceleration trim within : ");
    Serial.print(myIMU.SelfTest[0],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: acceleration trim within : ");
    Serial.print(myIMU.SelfTest[1],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: acceleration trim within : ");
    Serial.print(myIMU.SelfTest[2],1); Serial.println("% of factory value");
    Serial.print("x-axis self test: gyration trim within : ");
    Serial.print(myIMU.SelfTest[3],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: gyration trim within : ");
    Serial.print(myIMU.SelfTest[4],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: gyration trim within : ");
    Serial.print(myIMU.SelfTest[5],1); Serial.println("% of factory value");

    // Calibrate gyro and accelerometers, load biases in bias registers
    myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);

    myIMU.initMPU9250();
    // Initialize device for active mode read of acclerometer, gyroscope, and
    // temperature
    Serial.println("MPU9250 initialized for active data mode....");

    // Read the WHO_AM_I register of the magnetometer, this is a good test of
    // communication
    byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX);
    Serial.print(" I should be "); Serial.println(0x48, HEX);

    // Get magnetometer calibration from AK8963 ROM
    myIMU.initAK8963(myIMU.magCalibration);
    // Initialize device for active mode read of magnetometer
    Serial.println("AK8963 initialized for active data mode....");
  	//  Serial.println("Calibration values: ");
    Serial.print("X-Axis sensitivity adjustment value ");
    Serial.println(myIMU.magCalibration[0], 2);
    Serial.print("Y-Axis sensitivity adjustment value ");
    Serial.println(myIMU.magCalibration[1], 2);
    Serial.print("Z-Axis sensitivity adjustment value ");
    Serial.println(myIMU.magCalibration[2], 2);
  } // if (c == 0x71)
  else
  {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);
    while(1) ; // Loop forever if communication doesn't happen
  }

	// Calibrate the magnetometer x-axis in place.

#define CALIBRATION_ITERATIONS (100)
	double calibrationTotal = 0.0;

	for (int i = 0; i < CALIBRATION_ITERATIONS; i++) {
		myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values
		myIMU.getMres();
		// User environmental x-axis correction in milliGauss, should be
		// automatically calculated
		myIMU.magbias[0] = X_AXIS_MAG_BIAS;
		myIMU.mx = (float)myIMU.magCount[0]*myIMU.mRes*myIMU.magCalibration[0] -
							 myIMU.magbias[0];
		calibrationTotal += myIMU.mx;
		delay(100);
	}

	xAxisRestValue = calibrationTotal / CALIBRATION_ITERATIONS;
	Serial.print("X-axis rest value: "); Serial.println(xAxisRestValue);

	Serial.println();
	Serial.println("Exiting Setup");
	Serial.println();
	Serial.println();
	Serial.println();
}

void loop()
{

	// BM[E,P]280 readings

	//Each loop, take a reading.
	//Start with temperature, as that data is needed for accurate compensation.
	//Reading the temperature updates the compensators of the other functions
	//in the background.
	if (printBME280) {

		Serial.print("BMP280: Temperature: ");
		Serial.print(mySensor.readTempC(), 2);
		Serial.println(" degrees C");

		Serial.print("BMP280: Temperature: ");
		Serial.print(mySensor.readTempF(), 2);
		Serial.println(" degrees F");

		Serial.print("BMP280: Pressure: ");
		Serial.print(mySensor.readFloatPressure(), 2);
		Serial.println(" Pa");

		Serial.print("BMP280: Altitude: ");
		Serial.print(mySensor.readFloatAltitudeMeters(), 2);
		Serial.println("m");

		Serial.print("BMP280: Altitude: ");
		Serial.print(mySensor.readFloatAltitudeFeet(), 2);
		Serial.println("ft");

	}	// print BME280

	// MPU9250 readings

	if (printMPU9250) {
		if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {

			myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values
			myIMU.getAres();

			// Now we'll calculate the accleration value into actual g's
			// This depends on scale being set
			myIMU.ax = (float)myIMU.accelCount[0]*myIMU.aRes; // - accelBias[0];
			myIMU.ay = (float)myIMU.accelCount[1]*myIMU.aRes; // - accelBias[1];
			myIMU.az = (float)myIMU.accelCount[2]*myIMU.aRes; // - accelBias[2];

			myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values
			myIMU.getGres();

			// Calculate the gyro value into actual degrees per second
			// This depends on scale being set
			myIMU.gx = (float)myIMU.gyroCount[0]*myIMU.gRes;
			myIMU.gy = (float)myIMU.gyroCount[1]*myIMU.gRes;
			myIMU.gz = (float)myIMU.gyroCount[2]*myIMU.gRes;

			myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values
			myIMU.getMres();
			// User environmental x-axis correction in milliGauss, should be
			// automatically calculated
			myIMU.magbias[0] = X_AXIS_MAG_BIAS;
			// User environmental x-axis correction in milliGauss TODO axis??
			myIMU.magbias[1] = Y_AXIS_MAG_BIAS;
			// User environmental x-axis correction in milliGauss
			myIMU.magbias[2] = Z_AXIS_MAG_BIAS;

			// Calculate the magnetometer values in milliGauss
			// Include factory calibration per data sheet and user environmental
			// corrections
			// Get actual magnetometer value, this depends on scale being set
			myIMU.mx = (float)myIMU.magCount[0]*myIMU.mRes*myIMU.magCalibration[0] -
								 myIMU.magbias[0];
			myIMU.my = (float)myIMU.magCount[1]*myIMU.mRes*myIMU.magCalibration[1] -
								 myIMU.magbias[1];
			myIMU.mz = (float)myIMU.magCount[2]*myIMU.mRes*myIMU.magCalibration[2] -
								 myIMU.magbias[2];
		} // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

		// Must be called before updating quaternions!
		myIMU.updateTime();

		// Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of
		// the magnetometer; the magnetometer z-axis (+ down) is opposite to z-axis
		// (+ up) of accelerometer and gyro! We have to make some allowance for this
		// orientationmismatch in feeding the output to the quaternion filter. For the
		// MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward
		// along the x-axis just like in the LSM9DS0 sensor. This rotation can be
		// modified to allow any convenient orientation convention. This is ok by
		// aircraft orientation standards! Pass gyro rate as rad/s
		//  MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  my,  mx, mz);
		// MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx*DEG_TO_RAD,
		//											 myIMU.gy*DEG_TO_RAD, myIMU.gz*DEG_TO_RAD, myIMU.my,
		// 											 myIMU.mx, myIMU.mz, myIMU.deltat);

		// Print mag values in degree/sec
		Serial.print("MPU9250: X-mag field: "); Serial.print(myIMU.mx);
		Serial.println(" mG ");
		// Serial.print("MPU9250: Y-mag field: "); Serial.print(myIMU.my);
		// Serial.println(" mG ");
		// Serial.print("MPU9250: Z-mag field: "); Serial.print(myIMU.mz);
		// Serial.println(" mG");
		Serial.print("MPU9250: x-axis rest value: "); Serial.println(xAxisRestValue);
		Serial.print("MPU9250: x-axis computed state difference: "); Serial.println(abs(myIMU.mx - xAxisRestValue));
		if (abs(myIMU.mx - xAxisRestValue) > DOOR_CLOSED_VARIATION) {
			Serial.println("MPU9250: Door is OPEN");
		} else {
			Serial.println("MPU9250: Door is closed");
		}

		myIMU.tempCount = myIMU.readTempData();  // Read the adc values
		// Temperature in degrees Centigrade
		myIMU.temperature = ((float) myIMU.tempCount) / 333.87 + 21.0;
		// myIMU.temperature = ((float) myIMU.tempCount) / 340.0 + 36.53;	// Alternate calculation -- results in slightly higher temperatures
		// Print temperature in degrees Centigrade
		Serial.print("MPU9250: Die temperature is ");  Serial.print(myIMU.temperature, 1);
		Serial.println(" degrees C");
		Serial.println();

	}	// print MPU9250

	delay(1000);

}

/*
  10DOF MPU6050 (accelerometer, gyroscope), HMC5883L Triple Axis Digital Compass, MS5611 (digital temperature and pressure sensor) - GY-86
*/

#include <SPI.h>
#include <SdFat.h>

// Set USE_SDIO to zero for SPI card access, this section deals with the SD card
#define USE_SDIO 1
/*
 * SD chip select pin.  Common values are:
 *
 * Arduino Ethernet shield, pin 4.
 * SparkFun SD shield, pin 8.
 * Adafruit SD shields and modules, pin 10.
 * Default SD chip select is the SPI SS pin.
 */
const uint8_t SD_CHIP_SELECT = SS;
/*
 * Set DISABLE_CHIP_SELECT to disable a second SPI device.
 * For example, with the Ethernet shield, set DISABLE_CHIP_SELECT
 * to 10 to disable the Ethernet controller.
 */
const int8_t DISABLE_CHIP_SELECT = -1;

#if USE_SDIO
// Use faster SdioCardEX
//SdFatSdioEX sd;
  SdFatSdio sd;
#else // USE_SDIO
  SdFat sd;
#endif  // USE_SDIO

#include "Wire.h"    /* imports the wire library for talking over I2C */
#include <SoftwareSerial.h>

#include <HMC5883L.h>
#include <MPU6050.h>
#include <MS5611.h>

HMC5883L compass;
MPU6050 mpu;
MS5611 ms5611;

#include <TinyGPS++.h>    /* This section loads the GPS library */
TinyGPSPlus gps;          /* Initialize the gps object */

/* On Teensy, the UART (real serial port) is always best to use. */
/* Unlike Arduino, there's no need to use NewSoftSerial because */
/* the "Serial" object uses the USB port, leaving the UART free. */
HardwareSerial Uart = HardwareSerial();

int led = 13;

File mySensorData; //Data object you will write your sesnor data to
File NMEASentences;

double referencePressure;

String inputData;

String newgps;
String oldgps;

float pitch = 0.0;
float roll = 0.0;
float yaw = 0.0;

// global variables needed to operate the Kalman Filter
double xt;
double Pt;

// Setup routine to intiate the different sensors, SD card and the bluetooth device
void setup()
{
  pinMode(led, OUTPUT);     // initialize the led
  
  Serial.begin(115200);       // initialize the serial port

  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_4G)) // initialize the MPU
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }

  while(!ms5611.begin()) // initialize the pressure and temperature sensor
  {
    Serial.println("Could not find a valid MS5611 sensor, check wiring!");
    delay(500);
  }

  // If you have GY-86 or GY-87 module to access HMC5883L you need to disable the I2C Master Mode and Sleep Mode, and enable I2C Bypass Mode
  mpu.setI2CMasterModeEnabled(false);
  mpu.setI2CBypassEnabled(true);
  mpu.setSleepEnabled(false);

  // Gyroscope calibration - find out how this is done
  mpu.calibrateGyro();
 
  // Set sensitivity - find out how this is done
  // mpu.setThreshold(3);
  
  while (!compass.begin()) // Initialize HMC5883L
  {
    Serial.println("Could not find a valid HMC5883L sensor, check wiring!");
    delay(500);
  }

  // Set measurement range
  compass.setRange(HMC5883L_RANGE_1_3GA);

  // Set measurement mode
  compass.setMeasurementMode(HMC5883L_CONTINOUS);

  // Set data rate
  compass.setDataRate(HMC5883L_DATARATE_30HZ);

  // Set number of samples averaged
  compass.setSamples(HMC5883L_SAMPLES_8);

  // Set calibration offset. See HMC5883L_calibration.ino
  compass.setOffset(0, 0);

  mpu.setGyroOffsetX(100);
  mpu.setGyroOffsetY(100);
  mpu.setGyroOffsetZ(100);
  
  // Get reference pressure for relative altitude (wait for 10 seconds until the pressure read stabilizes
  referencePressure = 0;
  for (int i = 0; i < 100; i++) {
    referencePressure = referencePressure + ms5611.readPressure();
    delay(100);
  }
  referencePressure = referencePressure / 100;

  // Check settings
  // checkSettings();

  Uart.begin(9600);                   // initialize the gps sensor (connected to PINs 0, 1)

  Serial3.begin(9600);                // initialize the Bluetooth (connected to PINs 10, 9)

  #if USE_SDIO
    // Serial.print("USE SDIO");
    if (!sd.begin()) {
      Serial.print("\ncardBegin failed");
      return;
    }
  #else  // USE_SDIO
    // Initialize at the highest speed supported by the board that is not over 50 MHz. Try a lower speed if SPI errors occur.
    // Serial.print("USE non-SDIO");
    if (!sd.begin(SD_CHIP_SELECT, SD_SCK_MHZ(50))) {
      Serial.print("cardBegin failed");
      return;
    }
  #endif  // USE_SDIO
  sd.chvol();
}

void loop() {
  // put your main code here, to run repeatedly:
  bool newData = false;
  char c;
  
  // define the string to be printed out
  inputData = "";
  int counter = 1;
  //Serial3.println(String(mpu.getAccelOffsetX()));
  //Serial3.println(String(mpu.getAccelOffsetY()));
  //Serial3.println(String(mpu.getAccelOffsetZ()));
  //Serial3.println(String(mpu.getGyroOffsetX()));
  //Serial3.println(String(mpu.getGyroOffsetY()));
  //Serial3.println(String(mpu.getGyroOffsetZ()));

  if (!mySensorData.open("GPSDataNew.txt", O_RDWR | O_APPEND)) {  // open file to save data to the SD card
    Serial.println("open failed");
  }
  if (!NMEASentences.open("NMEASentences.txt", O_RDWR | O_APPEND)) {  // open file to save data to the SD card
    Serial.println("open failed");
  }

  // Wait for the first gps fix
  // Every second or so we parse GPS data and read the input
  for (unsigned long start = millis(); millis() - start < 1000;) {
    if (counter == 1) {
       digitalWrite(led, HIGH); // blink the led
       // read the gps
       while (Uart.available() > 0) {
          c = Uart.read();
          Serial3.write(c); // uncomment this line if you want to see the GPS data flowing
          NMEASentences.write(c);
          if (gps.encode(c))  // Did a new valid sentence come in?
            newData = true;
       }
       NMEASentences.close();
       if (newData) {
          newgps = "";
          newgps = newgps + String(gps.date.value()) + "," + String(gps.time.value()) + ",";      
          newgps = newgps + String(gps.location.lng(), 6) + "," + String(gps.location.lat(), 6) + ",";
          newgps = newgps + String(gps.altitude.meters()) + "," + String(gps.speed.kmph()) + ",";
          newgps = newgps + String(gps.course.deg()) + ",";
          newgps = newgps + String(gps.satellites.value()) + "," + String(gps.hdop.value()) + ",";
          if (newgps != "") {
            Serial3.println();
            readMPUreturnData(newgps);
            oldgps = newgps;
          }
        }
        else {
          if (oldgps != "") {
            Serial3.println();
            readMPUreturnData(oldgps);
          }
          else {
            oldgps = "00000,000000,0.000000,0.000000,0.00,0.00,0.00,0,0,"; // change this to place zeros only if GPS not read for 10 seconds
            readMPUreturnData(oldgps);          
          }
        }
    } // end of the counter == 1 if statement
    for (unsigned long start2 = millis(); millis() - start2 < 100;) {
      counter = counter + 1;
      readMPUreturnData(oldgps);
      Serial3.println(inputData);
      mySensorData.println(inputData);
      mySensorData.close();
      delay(50);
      digitalWrite(led, LOW);
    }
  }
}


// readMPUreturnData reads accelerometer, gyroscope, mageneto, pressure and temp data and creates the inputData content. 
// it takes a gps read if it's available or a string of zeroes if no available
String readMPUreturnData(String gpsRead){

  inputData = gpsRead;  // write gpsRead, either new read or old read or zeros (depending on the quality of the gps signal)
  
  double read_timeG;
  double read_timeA;
  double timer_start;
  
  // define the normalized compass read variable
  Vector norm = compass.readNormalize();
  
  // Calculate heading
  float heading = atan2(norm.YAxis, norm.XAxis);

  // Set declination angle on your location and fix heading
  // You can find your declination on: http://magnetic-declination.com/
  // (+) Positive or (-) for negative
  // For Bytom / Poland declination angle is 4'26E (positive)
  // Formula: (deg + (min / 60.0)) / (180 / M_PI);
  float declinationAngle = (4.0 + (26.0 / 60.0)) / (180 / M_PI);
  heading += declinationAngle;

  // Correct for heading < 0deg and heading > 360deg
  if (heading < 0)
  { heading += 2 * PI;}
  if (heading > 2 * PI)
  { heading -= 2 * PI;}

  // Convert to degrees
  float headingDegrees = heading * 180/M_PI; 

  // Output
  inputData = inputData + String(heading) + ",";
  inputData = inputData + String(headingDegrees) + ",";

  Vector rawGyro = mpu.readRawGyro(); /* Vector to hold raw gyro reads */
  // Vector normGyro = mpu.readNormalizeRawGyro(); /* Vector to hold raw gyro reads */
  
  timer_start = micros();
  inputData = inputData + String(rawGyro.XAxis) + ',';
  inputData = inputData + String(rawGyro.YAxis) + ',';
  inputData = inputData + String(rawGyro.ZAxis) + ',';
  //inputData = inputData + String(normGyro.XAxis) + ",";
  //inputData = inputData + String(normGyro.YAxis) + ",";
  //inputData = inputData + String(normGyro.ZAxis) + ",";
  read_timeG = micros() - timer_start;
  
  Vector rawAccel = mpu.readRawAccel();
  // Vector normGyro = mpu.readNormalizeAccel(); /* Vector to hold raw gyro reads */
    
  timer_start = micros();
  inputData = inputData + String(rawAccel.XAxis) + ",";
  inputData = inputData + String(rawAccel.YAxis) + ",";
  inputData = inputData + String(rawAccel.ZAxis) + ",";
  //inputData = inputData + String(normAccel.XAxis) + ",";
  //inputData = inputData + String(normAccel.YAxis) + ",";
  //inputData = inputData + String(normAccel.ZAxis) + ",";
  read_timeA = micros() - timer_start;
  
  // int pitch = -(atan2(normAccel.XAxis, sqrt(normAccel.YAxis*normAccel.YAxis + normAccel.ZAxis*normAccel.ZAxis))*180.0)/M_PI;
  // int roll = (atan2(normAccel.YAxis, normAccel.ZAxis)*180.0)/M_PI;
  
  // This section pertains to the pressure and temperature sensor readings - something worth studying is rawTemp and rawPressure readings
  // to improve the conversion of raw values into realTemp and realPressure

  // This section reads converted values
  double realTemperature = ms5611.readTemperature();
  long realPressure = ms5611.readPressure();  
 
  float absoluteAltitude = ms5611.getAltitude(realPressure);
  float relativeAltitude = ms5611.getAltitude(realPressure, referencePressure);
 
  inputData = inputData + String(realTemperature) + ",";
  inputData = inputData + String(realPressure) + ",";

  // how is pressure converted into absoluteAltitute
  inputData = inputData + String(absoluteAltitude) + ",";
  inputData = inputData + String(relativeAltitude) + ",";

  inputData = inputData + String(read_timeG) + ",";
  inputData = inputData + String(read_timeA);
  
  return inputData;
}


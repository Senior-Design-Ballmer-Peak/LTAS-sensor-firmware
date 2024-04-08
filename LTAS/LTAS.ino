// L-TAS Firmware
// Author: Charles Wilmot & Tristan Pawlenty

/* Include Statements */
#include <Wire.h>
#include <Arduino.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include "ICM_20948.h"
#include <SparkFunBME280.h>
#include <SPI.h>
#include <RH_RF69.h>

/* Class instances */
SFE_UBLOX_GNSS myGNSS;
ICM_20948_I2C myIMU;
BME280 myBME;

/* Radio Definitions */
#define RF69_FREQ 915.0
#define RFM69_CS    5
#define RFM69_INT   15
#define RFM69_RST   16

/* Singleton instance of the radio driver */
RH_RF69 rf69(RFM69_CS, RFM69_INT);

/* packet counter incremented per xmission */
int16_t packetnum = 0;

/* Function Prototype */
String createRadioPacket(float accX, float accY, float accZ, float gyrX, float gyrY, float gyrZ, float magX, float magY, float magZ, float tempC, float tempF, float humidity, float pressure, float latitude, float longitude, float altitude);
float convertDegMinToDecDeg(float degMin);

float accX = 0.0;
float accY = 0.0;
float accZ = 0.0;
float gyrX = 0.0;
float gyrY = 0.0;
float gyrZ = 0.0;
float magX = 0.0;
float magY = 0.0;
float magZ = 0.0;
float tempC = 0.0;
float tempF = 0.0;
float humidity = 0.0;
float pressure = 0.0;
float latitude = 0.0;
float longitude = 0.0;
float altitude = 0.0;

void setup() {
  /* Serial Setup */
  Serial.begin(115200);

  /* Wire Setup */
  Wire.begin();

  /* GPS Setup */
  if (myGNSS.begin() == false) {
    Serial.println(F("u-blox GNSS module not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }

  delay(2000);
  myGNSS.setI2COutput(COM_TYPE_UBX | COM_TYPE_NMEA); // Set the I2C port to output both NMEA and UBX messages
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); // Save (only) the communications port settings to flash and BBR
  
  delay(2000);
  //myGNSS.setNMEAOutputPort(Serial); // Configure the GNSS module to output NMEA sentences to the serial port

  /* IMU Setup */
  if (myIMU.begin() != ICM_20948_Stat_Ok) {
    Serial.println("ICM-20948 IMU not detected. Please check wiring. Freezing.");
    while (1);
  }

  /* BME Setup */
  if (!myBME.begin()) {
    Serial.println("BME280 sensor not detected. Please check wiring. Freezing.");
    while (1);
  }

  /* Radio Setup*/
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  /* Manual Reset */
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);

  /* RFM69 Setup */
  if (!rf69.init()) {
    Serial.println("RFM69 radio init failed");
    while (1);
  }

  /* Set Frequency */
  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency failed");
  }

  /* High power mode, true for 69hcw */
  rf69.setTxPower(20, true);

  // The encryption key has to be the same as the one in the server
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  rf69.setEncryptionKey(key);
}

void loop(){
  // delay(2000);

  if (myIMU.dataReady()){
    /* Get accelerometer, gyroscope, and magnetometer */
    myIMU.getAGMT();

    /* Accelerometer Data */
    accX = myIMU.accX();
    accY = myIMU.accY();
    accZ = myIMU.accZ();

    /* Gyroscopic Data */
    gyrX = myIMU.gyrX();
    gyrY = myIMU.gyrY();
    gyrZ = myIMU.gyrZ();

    /* Magnetometer Data */
    magX = myIMU.magX();
    magY = myIMU.magY();
    magZ = myIMU.magZ();
  }

  /* Temperature Data */
  tempC = myBME.readTempC();
  tempF = myBME.readTempF();

  /* Humidity Data */
  humidity = myBME.readFloatHumidity();

  /* Pressure Data */
  pressure = myBME.readFloatPressure();

  /* GPS Data */
  if (myGNSS.getGnssFixOk()) {
    latitude = float(myGNSS.getLatitude()) / 10000000.0;
    longitude = float(myGNSS.getLongitude()) / 10000000.0;
    altitude = float(myGNSS.getAltitude())  / 1000.0;
  } else {
    Serial.println("Waiting for GNSS fix...");
  }

  String packet = createRadioPacket(accX, accY, accZ, gyrX, gyrY, gyrZ, magX, magY, magZ, tempC, tempF, humidity, pressure, latitude, longitude, altitude);
  Serial.println(packet);

  unsigned int packetSize = RH_RF69_MAX_MESSAGE_LEN - 10; // Ensure this is an unsigned int
  unsigned int totalPackets = packet.length() / packetSize + ((packet.length() % packetSize) != 0 ? 1 : 0); // Use unsigned int here as well

  for (unsigned int i = 0; i < totalPackets; i++) {
    unsigned int start = i * packetSize;
    unsigned int end = min((i + 1) * packetSize, packet.length()); // Now both arguments are unsigned int
    
    String chunk = packet.substring(start, end);
    String header = "P" + String(i) + "of" + String(totalPackets) + ":";
    String message = header + chunk;
    
    // Convert the packet to a char array
    char radiopacket[message.length() + 1];
    message.toCharArray(radiopacket, sizeof(radiopacket));
    
    // Send the packet
    rf69.send((uint8_t *)radiopacket, strlen(radiopacket));
    rf69.waitPacketSent();
    delay(100); // Short delay to prevent overwhelming the receiver
  }

  delay(500);
}

String createRadioPacket(float accX, float accY, float accZ, float gyrX, float gyrY, float gyrZ, float magX, float magY, float magZ, float tempC, float tempF, float humidity, float pressure, float latitude, float longitude, float altitude) {
  String packet = "";

  // Accelerometer data (in m/s^2)
  packet += "AccX: " + String(accX) + " m/s^2, ";
  packet += "AccY: " + String(accY) + " m/s^2, ";
  packet += "AccZ: " + String(accZ) + " m/s^2, ";

  // Gyroscope data (in deg/s)
  packet += "GyrX: " + String(gyrX) + " deg/s, ";
  packet += "GyrY: " + String(gyrY) + " deg/s, ";
  packet += "GyrZ: " + String(gyrZ) + " deg/s, ";

  // Magnetometer data (in uT)
  packet += "MagX: " + String(magX) + " uT, ";
  packet += "MagY: " + String(magY) + " uT, ";
  packet += "MagZ: " + String(magZ) + " uT, ";

  // Temperature data (in °C and °F)
  packet += "TempC: " + String(tempC) + " °C, ";
  packet += "TempF: " + String(tempF) + " °F, ";

  // Humidity (in %)
  packet += "Humidity: " + String(humidity) + " %, ";

  // Pressure (in hPa)
  packet += "Pressure: " + String(pressure / 100.0, 2) + " hPa, ";

  // GPS data (in degrees and meters)
  if (latitude != 0.0 && longitude != 0.0) {
    packet += "Latitude: " + String(latitude, 8) + " degrees, ";
    packet += "Longitude: " + String(longitude, 8) + " degrees, ";
    packet += "Altitude: " + String(altitude, 3) + " meters";
  } else {
    packet += "Latitude: N/A, ";
    packet += "Longitude: N/A, ";
    packet += "Altitude: N/A";
  }

  Serial.println(packet);
  return packet;
}

float convertDegMinToDecDeg(float degMin) {
  int degrees = (int)(degMin / 100);
  float minutes = degMin - (degrees * 100);
  return degrees + (minutes / 60.0);
}

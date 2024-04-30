#include <Wire.h>
#include <Arduino.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> // For GNSS functionality
#include "ICM_20948.h" // For IMU functionality
#include <SparkFunBME280.h> // For environmental sensor functionality
#include <SPI.h>
#include <RH_RF69.h> // For RF communication

// Sensor and module instances
SFE_UBLOX_GNSS myGNSS; // GNSS module
ICM_20948_I2C myIMU; // IMU sensor
BME280 myBME; // Environmental sensor

// RF69 module configuration
#define RF69_FREQ 915.0
#define RFM69_CS 5
#define RFM69_INT 15
#define RFM69_RST 16

RH_RF69 rf69(RFM69_CS, RFM69_INT); // RF69 instance

int16_t packetnum = 0; // Packet number counter for RF transmission

// Function to calculate altitude from pressure readings
float calculatePressureBasedAltitude(float pressure, float seaLevelPressure = 1005) {
    return 44330.0 * (1.0 - pow(pressure / seaLevelPressure, 0.1903));
}

void setup() {
  Serial.begin(115200); // Start serial communication at 115200 baud rate
  Wire.begin(); // Initialize I2C communication

  // Check and initialize GNSS module
  if (!myGNSS.begin()) {
    Serial.println("u-blox GNSS module not detected at default I2C address. Please check wiring. Freezing.");
    while (1); // Freeze if module not detected
  }

  // Configure GNSS output type and save configuration
  myGNSS.setI2COutput(COM_TYPE_UBX | COM_TYPE_NMEA);
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);

  // Check and initialize IMU sensor
  if (myIMU.begin() != ICM_20948_Stat_Ok) {
    Serial.println("ICM-20948 IMU not detected. Please check wiring. Freezing.");
    while (1); // Freeze if sensor not detected
  }

  // Check and initialize environmental sensor
  if (!myBME.begin()) {
    Serial.println("BME280 sensor not detected. Please check wiring. Freezing.");
    while (1); // Freeze if sensor not detected
  }

  // Reset RF69 module
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);

  // Initialize RF69 radio
  if (!rf69.init()) {
    Serial.println("RFM69 radio init failed");
    while (1); // Freeze if radio fails to initialize
  }

  // Set RF69 frequency
  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency failed");
  }

  // Set RF69 transmission power
  rf69.setTxPower(20, true);

  // Set encryption key for RF69
  uint8_t key[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                   0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  rf69.setEncryptionKey(key);
}

void loop() {
  if (myIMU.dataReady()) {
    myIMU.getAGMT(); // Get all new data from IMU
    // Convert acceleration from milli-g to m/s^2
    float accX = myIMU.accX() * 9.80665 / 1000;
    float accY = myIMU.accY() * 9.80665 / 1000;
    float accZ = myIMU.accZ() * 9.80665 / 1000;
    // Read gyroscope data
    float gyrX = myIMU.gyrX();
    float gyrY = myIMU.gyrY();
    float gyrZ = myIMU.gyrZ();
    // Read environmental data
    float tempF = myBME.readTempF();
    float humidity = myBME.readFloatHumidity();
    float pressure = myBME.readFloatPressure() / 100.0; // Convert Pa to hPa
    float altitudeFromPressure = calculatePressureBasedAltitude(pressure);
    // Initialize GNSS data variables
    float latitude = 0.0, longitude = 0.0, altitude = 0.0;

    if (myGNSS.getGnssFixOk()) {
      latitude = myGNSS.getLatitude() / 10000000.0;
      longitude = myGNSS.getLongitude() / 10000000.0;
      altitude = myGNSS.getAltitude() / 1000.0; // Convert mm to m
    }

    // Calculate average altitude from GNSS and pressure sensor
    float newAltitude = (altitude + altitudeFromPressure) / 2;

    // Create and send radio packet with all sensor data
    String packet = createRadioPacket(accX, accY, accZ, gyrX, gyrY, gyrZ, tempF, humidity, pressure, latitude, longitude, newAltitude);
    sendRadioPacket(packet);
  }
}

// Function to create a formatted string with sensor data
String createRadioPacket(float accX, float accY, float accZ, float gyrX, float gyrY, float gyrZ, float tempF, float humidity, float pressure, float latitude, float longitude, float altitude) {
  char buffer[400];
  snprintf(buffer, 400,
           "AccX: %0.2f, AccY: %0.2f, AccZ: %0.2f, "
           "GyrX: %0.2f, GyrY: %0.2f, GyrZ: %0.2f, "
           "TempF: %0.2f, Humidity: %0.2f, "
           "Pressure: %0.2f, Latitude: %0.2f, Longitude: %0.2f, Altitude: %0.2f",
           accX, accY, accZ, gyrX, gyrY, gyrZ, tempF, humidity, pressure, latitude, longitude, altitude);
  return String(buffer);
}

// Function to send the data packet via RF69
void sendRadioPacket(String packet) {
  unsigned int packetSize = RH_RF69_MAX_MESSAGE_LEN - 10;
  unsigned int totalPackets = packet.length() / packetSize + ((packet.length() % packetSize) != 0 ? 1 : 0);

  for (unsigned int i = 0; i < totalPackets; i++) {
    unsigned int start = i * packetSize;
    unsigned int end = min((i + 1) * packetSize, packet.length());
    
    String chunk = packet.substring(start, end);
    String header = "P" + String(i) + "of" + String(totalPackets) + ":";
    String message = header + chunk;
    
    char radiopacket[message.length() + 1];
    message.toCharArray(radiopacket, sizeof(radiopacket));
    
    rf69.send((uint8_t *)radiopacket, strlen(radiopacket)); // Send the packet
    rf69.waitPacketSent(); // Wait for the packet to be sent
    delay(10); // Short delay between packets
  }
}

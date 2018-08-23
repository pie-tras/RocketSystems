#include <SPI.h>
#include <RH_RF95.h>

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>

#define RFM95_CS 4
#define RFM95_RST 1
#define RFM95_INT 3

#define RF95_FREQ 915.0

#define BMP_SCK 7
#define BMP_SDO 8
#define BMP_SDI 9
#define BMP_CS 10

RH_RF95 rf95(RFM95_CS, RFM95_INT);

Adafruit_BMP280 bmp(BMP_CS, BMP_SDI, BMP_SDO, BMP_SCK);

Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);


void displaySensorDetails(void)
{
  sensor_t sensor;
  accel.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" m/s^2");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" m/s^2");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" m/s^2");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void setup() {
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  while (!Serial);
  Serial.begin(9600);
  delay(100);

  Serial.println("Booting Flight Systems...");

  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }
  Serial.println("LoRa radio init was sucessful");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);

  Serial.println(F("BMP280 init..."));

  if (!bmp.begin()) {
    Serial.println(F("BMP280 sensor not detected, check wiring!"));
    while (1);
  }

  if(!accel.begin())
  {
    Serial.println("Accel-LSM303 not detected, Check wiring!");
    while(1);
  }

  displaySensorDetails();

  mag.enableAutoRange(true);

  if(!mag.begin())
  {
    Serial.println("Mag-LSM303 not detected, Check wiring!");
    while(1);
  }

  displaySensorDetails();
  
}

int resolution = 100;

float alt = 0;
float temp = 0;
float pa = 0;

float accX = 0;
float accY = 0;
float accZ = 0;

float magX = 0;
float magY = 0;
float magZ = 0;

void loop() {
  
  sensors_event_t event;
  accel.getEvent(&event);  
  
  accX = event.acceleration.x;
  accY = event.acceleration.y;
  accZ = event.acceleration.z;

  mag.getEvent(&event);

  magX = event.magnetic.x;
  magY = event.magnetic.y;
  magZ = event.magnetic.z;

  alt = bmp.readAltitude(1000);
  temp = bmp.readTemperature();
  pa = bmp.readPressure();

  char accPacket[30] = "X         Y         Z        ";
  itoa(accX * resolution, accPacket + 1, 10);
  itoa(accY * resolution, accPacket + 12, 10);
  itoa(accZ * resolution, accPacket + 22, 10);
  
  Serial.println("Sending accPkt");
  rf95.send((uint8_t *)accPacket, 30);
  
  rf95.waitPacketSent();
  // Now wait for a reply
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  Serial.println("Waiting for reply...");
  delay(10);
  if (rf95.waitAvailableTimeout(1000)) {
    // Should be a reply message for us now
    if (rf95.recv(buf, &len)) {
      Serial.print("Got reply: ");
      Serial.println((char*)buf);
      Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);
    } else {
      Serial.println("Receive failed");
    }
  } else {
    Serial.println("No reply, is there a listener in range?");
  }
  delay(1000);
}

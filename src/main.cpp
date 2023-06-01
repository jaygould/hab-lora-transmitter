#include <Arduino.h>
#include <RadioLib.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SPI.h>
#include "SD.h"
#include "FS.h"
#include "LoraMessage.h"

// SD
SPIClass spi = SPIClass(VSPI);

// Sleep
#define uS_TO_S_FACTOR 1000000
#define TIME_TO_SLEEP  20
RTC_DATA_ATTR int bootCount = 0;

// ESP32 slave
#define HSPI_SS_PIN 15
SPIClass *hspi = NULL;

// LoRa
SX1278 radio = new Module(4, 26, 2);

// GPS
int RXPin = 1;
int TXPin = 0;
int GPSBaud = 9600;
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);

long lastDisplayTime = 0;
float lat = 0;
float lng = 0;
float alt = 0;
float speed = 0;
char timeString[32] = "00:00:00";

// Temp
const int oneWireBus = 32;     
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);
float temperatureC = 0;

// General
char callSign[] = "FINN0137";

// Options
boolean showOnSondehubMap = true;
boolean enableSleepRunMode = false;

// Declarations (PlatformIO only)
void transmitLocation(char* data);
char * getTimeStr(TinyGPSTime &t);
void sendToEsp32Slave(SPIClass *spi, byte data[], int length);
void appendFile(fs::FS &fs, const char * path, const char * message);

void printHex(uint8_t num) {
  char hexCar[2];

  sprintf(hexCar, "%02X", num);
  Serial.print(hexCar);
}

// Begin
void setup() {
  Serial.begin(115200);
  delay(1000);

  // Sleep - config
  if(enableSleepRunMode) {
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) + " Seconds");
    Serial.println("Boot number: " + String(bootCount));
  }
  
  // ESP32 slave
  Serial.println("Seting up SPI");
  hspi = new SPIClass(HSPI);
  hspi->begin();
  pinMode(HSPI_SS_PIN, OUTPUT);

  // GPS
  gpsSerial.begin(GPSBaud, SERIAL_8N1, 17, 16);

  // Temp
  sensors.begin();

  // LoRa
  Serial.print("[SX1278] Initializing .. ");
  int state = radio.begin(433);
  radio.explicitHeader();
  radio.setBandwidth(31.25);
  radio.setSpreadingFactor(7);
  radio.setCodingRate(8);
  radio.setOutputPower(4); // TODO: change back to 18 or 20
  radio.setCRC(true);

  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("success!");
  } else {
    Serial.println("failed, code ");
    Serial.print(state);
    while (true);
  }

  // SD card - must go at the bottom of the setup() for some reason, otherwise the card doesn't mount 
  spi.begin(18, 19, 23, 5);
  if(!SD.begin(5, spi)){
    Serial.println("Card Mount Failed");
    // return;
  }
  
  uint8_t cardType = SD.cardType();

  if(cardType == CARD_NONE){
    Serial.println("No SD card attached");
    return;
  }

  Serial.print("SD Card Type: ");
  if(cardType == CARD_MMC){
    Serial.println("MMC");
  } else if(cardType == CARD_SD){
    Serial.println("SDSC");
  } else if(cardType == CARD_SDHC){
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }

  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);
}

void loop() {
  while(gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  if(millis() - lastDisplayTime >= 10000) {
    // Get temperature
    sensors.requestTemperatures(); 
    temperatureC = sensors.getTempCByIndex(0);
    Serial.println("Temperature:");
    Serial.println(temperatureC);
    Serial.print("ºC");

    if (gps.location.isValid() && gps.altitude.isValid() && gps.speed.isValid()) {
      // Send GPS data to slave ESP32 for transmission to LoRaWAN
      // TODO: convert GPS to byte data?
      // needs to be defined in the loop and NOT outside, otherwise it keeps adding to the variable and causes crashing when 
      // trying to read illegal parts of memory
      LoraMessage message;
      
      message
        .addLatLng(gps.location.lat(), gps.location.lng());

      // * View contents of byte array if needed...
      // int i;
      // for(i=0; i < message.getLength(); i++){
      //   printHex(message.getBytes()[i]);
      // }

      // uint8_t buffer[] = { 0xFF, 0xF0, 0x0F, 0x11 };
      sendToEsp32Slave(hspi, message.getBytes(), message.getLength());


      // Get the latest info from the gps object which it derived from the data sent by the GPS unit
      Serial.println("Satellite Count:");
      Serial.println(gps.satellites.value());
      Serial.println("Latitude:");
      Serial.println(gps.location.lat(), 6);
      Serial.println("Longitude:");
      Serial.println(gps.location.lng(), 6);
      Serial.println("Speed MPH:");
      Serial.println(gps.speed.mph());
      Serial.println("Altitude Feet:");
      Serial.println(gps.altitude.feet());
      Serial.println("");

      lat = gps.location.lat();
      lng = gps.location.lng();
      alt = gps.altitude.feet();
      speed = gps.speed.mph();

      if(gps.time.isValid()) {
        sprintf(timeString, "%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
      } else {
        sprintf(timeString, "%s", "00:00:00");
      }
    }

    // TODO: maybe send byte array instead of string? depends what LoRa-gateway can track
    char locationString[256];
    sprintf(locationString, "%s%s%s%i%s%s%s%.4f%s%.4f%s%.2f%s%.2f%s%.2f", showOnSondehubMap ? "$$" : "", callSign, ",", bootCount, ",", timeString, ",", lat, ",", lng, ",", alt, ",", speed, ",", temperatureC);

    transmitLocation(locationString);
    appendFile(SD, "/telemetry.txt", locationString);

    lastDisplayTime = millis();

    // TODO: maybe only sleep once GPS is valid and has been sent?
    if(enableSleepRunMode) {
      ++bootCount;
      Serial.println("Going to sleep now");
      delay(1000);
      esp_deep_sleep_start();
    } else {
      bootCount++;
    }
  }
}

void transmitLocation(char* data) {
  Serial.println("Transmitting packet ... ");
  Serial.println(data);
  int state = radio.transmit(data);

  if (state == RADIOLIB_ERR_NONE) {
    // the packet was successfully transmitted
    Serial.println(F("Packet transmitted ok!"));

    // print measured data rate
    Serial.print(F("[SX1278] Datarate: "));
    Serial.print(radio.getDataRate());
    Serial.println(F(" bps"));

  } else if (state == RADIOLIB_ERR_PACKET_TOO_LONG) {
    // the supplied packet was longer than 256 bytes
    Serial.println(F("too long!"));

  } else if (state == RADIOLIB_ERR_TX_TIMEOUT) {
    // timeout occurred while transmitting packet
    Serial.println(F("timeout!"));

  } else {
    // some other error occurred
    Serial.print(F("failed, code "));
    Serial.println(state);
  }
}

void sendToEsp32Slave(SPIClass *spi, byte data[], int length) {
  //use it as you would the regular arduino SPI API
  spi->beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(spi->pinSS(), LOW); //pull SS slow to prep other end for transfer
  spi->transfer(data, length);
  digitalWrite(spi->pinSS(), HIGH); //pull ss high to signify end of data transfer
  spi->endTransaction();
}

void appendFile(fs::FS &fs, const char * path, const char * message){
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if(!file){
    Serial.println("Failed to open file for appending");
    return;
  }
  if(file.print(message)){
    Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();
}

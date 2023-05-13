#include <Arduino.h>
#include <RadioLib.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// Sleep
#define uS_TO_S_FACTOR 1000000
#define TIME_TO_SLEEP  20

RTC_DATA_ATTR int bootCount = 0;


// LoRa
SX1278 radio = new Module(4, 26, 2);

// GPS
int RXPin = 1;
int TXPin = 0;
int GPSBaud = 9600;
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);
// SoftwareSerial gpsSerial(RXPin, TXPin);

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
boolean showOnSondehubMap = true;
char callSign[] = "FINN0137";

void transmitLocation(char* data);
char * getTimeStr(TinyGPSTime &t);

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Sleep - config
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) + " Seconds");
  
  // Sleep - increment boot number and print it every reboot
  ++bootCount;
  Serial.println("Boot number: " + String(bootCount));

  // GPS
  gpsSerial.begin(GPSBaud, SERIAL_8N1, 17, 16);
  // gpsSerial.begin(GPSBaud);


  // Temp
  sensors.begin();

  // LoRa
  Serial.print("[SX1278] Initializing .. ");
  int state = radio.begin(433);
  radio.explicitHeader();
  radio.setBandwidth(31.25);
  radio.setSpreadingFactor(11);
  radio.setCodingRate(8);
  radio.setOutputPower(20);
  radio.setCRC(true);

  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("success!");
  } else {
    Serial.println("failed, code ");
    Serial.print(state);
    while (true);
  }
}

void loop() {
  while(gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  if(millis() - lastDisplayTime >= 5000) {
    // Get temperature
    sensors.requestTemperatures(); 
    temperatureC = sensors.getTempCByIndex(0);
    Serial.println("Temperature:");
    Serial.println(temperatureC);
    Serial.print("ºC");

    if (gps.location.isValid() && gps.altitude.isValid() && gps.speed.isValid()) {

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

    char locationString[256];
    sprintf(locationString, "%s%s%s%i%s%s%s%.2f%s%.2f%s%.2f%s%.2f%s%.2f", showOnSondehubMap ? "$$" : "", callSign, ",", bootCount, ",", timeString, ",", lat, ",", lng, ",", alt, ",", speed, ",", temperatureC);

    transmitLocation(locationString);

    lastDisplayTime = millis();

    // TODO: maybe only sleep once GPS is valid and has been sent?
    Serial.println("Going to sleep now");
    delay(1000);
    esp_deep_sleep_start();
    
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

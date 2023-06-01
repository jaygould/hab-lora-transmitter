# HAB LoRa Transmitter

Program for loading on to an ESP32 for transmitting telemetry data from a high altitude balloon payload.

## Related repos

- [HAB API](https://github.com/jaygould/hab-api)
- [HAB React Native app](https://github.com/jaygould/hab-app)
- [HAB LoRaWAN transmitter](https://github.com/jaygould/hab-lorawan-transmitter)

## Components 

Uses an ESP32 microcontroller with various components wired in to transmit telemetry and sensor data to the ground using LoRa radio. Components include:

- ESP32 microcontroller - main board which sends/receives data between attached components
- LoRa transceiver - sends telemetry data over radio to be received by other LoRa transceivers within range
- GPS receiver - receives location and altitude data (as well as current date/time) for tracking position of HAB
- Temperature sensor - gets temperature from waterproof external sensor
- SD card reader - saves telemetry data to SD card for when transmission is out of range or otherwise not received
- SPI connection to second ESP32 module - send telemetry data to second ESP32 module


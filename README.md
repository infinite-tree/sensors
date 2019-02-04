# Simple ESP32 Data collection platform

## Supported Sensors

  - BME280 (Temp, humidity, Pressure)
  - Adafruit Anemometer
  - Cheap 0.3mm tilt based rain gauge


## Building & Installing

I use platformIO, once installed, it should read the platformio.ini file for the rest.
You'll also need to install my fork of the arduino [ConfigTool](https://github.com/infinite-tree/ConfigTool) library. A stupid way to do this is symlink the ConfigTool.h and .cpp files into your local src directory.


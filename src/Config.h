/*

    Config.h

    Simple config loading and saving based (ripped off) from the examples in ArduinoJson
*/

#ifndef _Config_h
#define _Config_h

#include <Arduino.h>


/* Sample Config:
{
    "Magic": "######",
    "WifiSSID": "XXXXXXXXXXXXXXXXXXXX",
    "WifiPassword": "XXXXXXXXXXXXXXXXXX",
    "Location": "XXXXXXXXXXXX",
    "Sensor": "XXXXXXXXXXX",
    "InfluxHostname": "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX",
    "InfluxDatabase": "XXXXXXXXXXXX",
    "InfluxUser": "XXXXXXXXXXX",
    "InfluxPaddword": "XXXXXXXXXXXX",
    "EnableTempSensor": false,
    "EnableWindSensor": false,
    "EnableRainSensor": false,
    "EnableSoilSensor": false
}

Paste this into https://arduinojson.org/v6/assistant/ for every change
*/

#define CONFIG_FILE     "/config.json"
#define CONFIG_SIZE     515
#define CONFIG_MAGIC    "1337"

struct Config
{
    String Magic;
    String WifiSSID;
    String WifiPassword;

    bool EnableSleepMode;

    String Location;
    String Sensor;

    String InfluxHostname;
    String InfluxDatabase;
    String InfluxUser;
    String InfluxPassword;

    bool EnableTempSensor;
    bool EnableWindSensor;
    bool EnableRainSensor;
    bool EnableSoilSensor;

};


void loadConfig(Config &config);
void saveConfig(Config &config);
void printConfig(Config &config);
void askForPreferences(Config &config);
void askForSettings(Config &config);

#endif
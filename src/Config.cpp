/*
    Config.cpp

    Simple config loading and saving based (ripped off) from the examples in ArduinoJson
*/

#include "Config.h"
#include <ArduinoJson.h>
#include <FS.h>
#include <SPIFFS.h>

void loadConfig(Config &config) {
    if (!SPIFFS.begin(true))
    {
        Serial.println("SPIFFS Failed");
    }
    File file = SPIFFS.open(CONFIG_FILE, "r");
    if (!file)
    {
        Serial.println("Failed to open config file");
        return;
    }
    // Allocate a temporary JsonDocument
    // Don't forget to change the capacity to match your requirements.
    // Use arduinojson.org/v6/assistant to compute the capacity.
    StaticJsonDocument<CONFIG_SIZE> doc;

    // Deserialize the JSON document
    DeserializationError error = deserializeJson(doc, file);
    if (error)
        Serial.println(F("Failed to read file, using default configuration"));

    // Copy values from the JsonDocument to the Config
    config.Magic = doc["Magic"].as<String>();

    config.WifiSSID = doc["WifiSSID"].as<String>();
    config.WifiPassword = doc["WifiPassword"].as<String>();

    config.Location = doc["Location"].as<String>();
    config.Sensor = doc["Sensor"].as<String>();

    config.InfluxHostname = doc["InfluxHostname"].as<String>();
    config.InfluxDatabase = doc["InfluxDatabase"].as<String>();
    config.InfluxUser = doc["InfluxUser"].as<String>();
    config.InfluxPassword = doc["InfluxPassword"].as<String>();

    config.EnableTempSensor = doc["EnableTempSensor"];
    config.EnableWindSensor = doc["EnableRainSensor"];
    config.EnableRainSensor = doc["EnableRainSensor"];
    config.EnableSoilSensor = doc["EnableSoilSensor"];

    file.close();
}



void saveConfig(Config &config) {
    SPIFFS.begin(true);
    SPIFFS.remove(CONFIG_FILE);

    // Open file for writing
    File file = SPIFFS.open(CONFIG_FILE, "w");
    if (!file)
    {
        Serial.println(F("Failed to create file"));
        return;
    }

    // Allocate a temporary JsonDocument
    // Don't forget to change the capacity to match your requirements.
    // Use arduinojson.org/assistant to compute the capacity.
    StaticJsonDocument<CONFIG_SIZE> doc;

    // Set the values in the document
    doc["Magic"] = config.Magic;

    doc["WifiSSID"] = config.WifiSSID;
    doc["WifiPassword"] = config.WifiPassword;

    doc["Location"] = config.Location;
    doc["Sensor"] = config.Sensor;

    doc["InfluxHostname"] = config.InfluxHostname;
    doc["InfluxDatabase"] = config.InfluxDatabase;
    doc["InfluxUser"] = config.InfluxUser;
    doc["InfluxPassword"] = config.InfluxPassword;

    doc["EnableTempSensor"] = config.EnableTempSensor;
    doc["EnableWindSensor"] = config.EnableWindSensor;
    doc["EnableRainSensor"] = config.EnableRainSensor;
    doc["EnableSoilSensor"] = config.EnableSoilSensor;

    // Serialize JSON to file
    if (serializeJson(doc, file) == 0)
    {
        Serial.println(F("Failed to write to file"));
    }

    // Close the file
    file.close();
}

void printBool(bool value)
{
    if (value)
    {
        Serial.println("Yes");
    }
    else
    {
        Serial.println("No");
    }
}

void printConfig(Config &config)
{
    Serial.println("########## Device Setting ##########");

    Serial.print("Wifi SSID: ");
    Serial.println(config.WifiSSID);
    Serial.print("Wifi Password: ");
    Serial.println(config.WifiPassword);

    Serial.println();

    Serial.print("Location: ");
    Serial.println(config.Location);
    Serial.print("Sensor Name: ");
    Serial.println(config.Sensor);

    Serial.println();

    Serial.print("Influxdb Host: ");
    Serial.println(config.InfluxHostname);
    Serial.print("Influxdb Database: ");
    Serial.println(config.InfluxDatabase);
    Serial.print("Influxdb Username: ");
    Serial.println(config.InfluxUser);
    Serial.print("Influxdb Password: ");
    Serial.println(config.InfluxPassword);

    Serial.println();

    Serial.print("Temp Sensor enabled: ");
    printBool(config.EnableTempSensor);
    Serial.print("Wind Sensor enabled: ");
    printBool(config.EnableWindSensor);
    Serial.print("Rain Sensor enabled: ");
    printBool(config.EnableRainSensor);
    Serial.print("Soil Sensor enabled: ");
    printBool(config.EnableSoilSensor);

    Serial.println();
}

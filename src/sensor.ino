// Built-in includes
#include <Arduino.h>
#include "ConfigTool.h"
#include <driver/adc.h>
#include <WiFi.h>

#include <Wire.h>
#include <SparkFunBME280.h>

// Downloaded from https://github.com/teebr/Influx-Arduino
#include <InfluxArduino.hpp>
#include "RootCert.hpp"


// Conversions
#define MPS_TO_MPH      2.23694
// Standard cheap tilt rainfall sensor is 0.3mm per tilt, (but it counts 4 times per tilt)
#define COUNT_TO_MM     0.3/4

// Sensor Pins
#define BME_SDA_PIN     21
#define BME_SDL_PIN     22
#define WIND_PIN        ADC1_CHANNEL_0 // 36
#define RAIN_PIN        22
#define RAIN_GND        23

// How often to send data in milliseconds
#define TEMP_HUMIDITY_DELAY         30 * 1000
#define WIND_DELAY                  5 * 1000
#define RAIN_DELAY                  10 * 1000


InfluxArduino influx;
BME280 bme;

// rain sensor variables
#define RAIN_SAMPLES                360
volatile uint8_t rainCounter = 0;
uint16_t rainHead = 0;
uint8_t rainSamples[RAIN_SAMPLES] = {0};
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// Wifi Settings
#define CONNECTION_RETRY        10

// Influx Settings

const char TEMP_MEASUREMENT[] = "temperature_fahrenheit";
const char HUMIDITY_MEASUREMENT[] = "humidity_percentage";
const char PRESSURE_MEASUREMENT[] = "pressure_hpa";
const char WIND_MEASUREMENT[] = "wind_mph";
const char RAIN_MEASUREMENT[] = "rain_mm_hr";

String Tags;

// Track the last time data was sent to influx
unsigned long tempHumidLastSent = 0;
unsigned long windLastSent = 0;
unsigned long rainLastSent = 0;

const char CONFIG_MAGIC[] = "1337";
struct Config {
    String Magic;
    String WifiSSID;
    String WifiPassword;

    String Location;
    String Sensor;

    String InfluxHost;
    String InfluxDatabase;
    String InfluxUser;
    String InfluxPassword;

    bool EnableTempSensor;
    bool EnableWindSensor;
    bool EnableRainSensor;
};
Config config;
ConfigTool configTool;


float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setupWindSensor() {
    // Setup wind sensor
    pinMode(WIND_PIN, INPUT);
    Serial.println("Wind Sensor initialized");
}

void sendWindSpeed() {
    // Hard coded for the Adafruit Anemometer

    // Set the ADC to 10 bits instead of default of 12 (range = 1024)
    adc1_config_width(ADC_WIDTH_BIT_10);
    // Set the attenuation of the Wind sensor pin to 0-2V
    adc1_config_channel_atten(WIND_PIN, ADC_ATTEN_DB_6);
    int16_t sensorValue1 = adc1_get_raw(WIND_PIN);
    delay(20);
    int16_t sensorValue2 = adc1_get_raw(WIND_PIN);
    delay(20);
    int16_t sensorValue3 = adc1_get_raw(WIND_PIN);
    delay(20);

    int16_t sensorValue = (sensorValue1 + sensorValue2 + sensorValue3)/3;

    float scaledValue = 0;
    Serial.print("ADC: ");
    Serial.println(sensorValue);

    // Scale value 0.4V = 0 m/s, 2.0V = 32.4 m/s
    // Measured voltages:
    // 2.0V = 1023 (power supply @2v)
    // 0.4V = 166 (sensor stationary)
    sensorValue = constrain(sensorValue, 166, 1024);
    scaledValue = mapf(float(sensorValue), 166, 1024, 0.0, 32.4);

    uint16_t value = scaledValue*MPS_TO_MPH;
    Serial.print("W = ");
    Serial.println(value);

    char fields[16];
    sprintf(fields, "value=%d", value);

    if (!influx.write(WIND_MEASUREMENT, Tags.c_str(), fields)) {
        Serial.print("ERROR sending wind: ");
        Serial.println(influx.getResponse());
    }
}


void setupRainSensor() {
    // Setup Rain sensor
    pinMode(RAIN_GND, OUTPUT);
    digitalWrite(RAIN_GND, LOW);
    pinMode(RAIN_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(RAIN_PIN), rainInterrupt, FALLING);
    Serial.println("Rain sensor initialized");
}

void IRAM_ATTR rainInterrupt() {
  portENTER_CRITICAL_ISR(&mux);
  rainCounter++;
  portEXIT_CRITICAL_ISR(&mux);
}

void sendRainfall() {
    // Convert to mm and send a moving 1hr value
    // The rain sensor interrupts 4 times every 0.3mm of water hits it
    // (which is already accounted for in COUNT_TO_MM)

    // lock to read/write the rain counter
    portENTER_CRITICAL(&mux);
    rainSamples[rainHead] = rainCounter;
    rainCounter = 0;
    rainHead = (rainHead + 1) % RAIN_SAMPLES;
    portEXIT_CRITICAL(&mux);

    // Calculate the sum over the last hour
    uint32_t sum = 0;
    for (uint16_t x=0; x<RAIN_SAMPLES; x++) {
        sum += rainSamples[x];
    }
    float mm = sum * COUNT_TO_MM;

    char fields[16];
    sprintf(fields, "value=%f", mm);
    if (!influx.write(RAIN_MEASUREMENT, Tags.c_str(), fields)) {
        Serial.print("ERROR sending rain: ");
        Serial.println(influx.getResponse());
    }
}

void setupBME280() {
    // Setup the BME280 (Temp, Humidity, and Pressure) sensor
    // hard coded i2c address... why?
    Wire.begin();
    bme.setI2CAddress(0x76);
    if (!bme.beginI2C()) {
        Serial.println("ERROR: Failed to initialize BME280");
        return;
    }
    Serial.println("BME280 initialized");
}

void sendTempAndHumidity() {
    float temperature = bme.readTempF();
    // float pressure = bme.readFloatPressure() / 100.0F;
    float humidity = bme.readFloatHumidity();

    Serial.print("T: ");
    Serial.println(temperature);
    Serial.print("H: ");
    Serial.println(humidity);
    // Serial.print("P: ");
    // Serial.println(pressure);

    // Setup Influx points to send
    char temp_fields[16];
    char humidity_fields[16];
    // char pressure_fields[20];
    sprintf(temp_fields, "value=%f", temperature);
    sprintf(humidity_fields, "value=%f", humidity);
    // sprintf(pressure_fields, "value=%f", pressure);

    if (!influx.write(TEMP_MEASUREMENT, Tags.c_str(), temp_fields)) {
        Serial.print("ERROR sending temp: ");
        Serial.println(influx.getResponse());
    }

    if (!influx.write(HUMIDITY_MEASUREMENT, Tags.c_str(), humidity_fields)) {
        Serial.print("ERROR sending humidity: ");
        Serial.println(influx.getResponse());
    }

    // if (!influx.write(PRESSURE_MEASUREMENT, Tags.c_str(), pressure_fields)) {
    //     Serial.print("ERROR sending pressure: ");
    //     Serial.println(influx.getResponse());
    // }

    Serial.println("Temp and Humidity sent!");
}

void connectToWifi() {
    while (true) {
        Serial.print("Connecting to Wifi network ");
        Serial.print(config.WifiSSID);
        Serial.print(" ");
        WiFi.scanNetworks();
        WiFi.begin(config.WifiSSID.c_str(), config.WifiPassword.c_str());
        for (int x=0; x<CONNECTION_RETRY; x++) {
            if (WiFi.status() == WL_CONNECTED) {
                Serial.println("Connected");
                return;
            }
            Serial.print(".");
            delay(500);
        }
    }
}

String readString() {
    String value = "";
    char c;
    while (true) {
        if (Serial.available()) {
            c = Serial.read();
            if (c == '\n' || c == '\r') {
                // clear anything else that might be waiting
                c = Serial.read();
                Serial.println();
                return value;
            } else {
                Serial.print(c);
                value += c;
            }
        }
        delay(3);
    }
}

bool readBool() {
    char c;
    bool ret = false;
    while (true) {
        if (Serial.available()) {
            c = Serial.read();
            if (c == 'y' || c== 'Y') {
                Serial.print(c);
                ret = true;
            } else if (c == 'n' || c == 'N') {
                Serial.print(c);
                ret = false;
            } else if (c == '\n' || c =='\r') {
                // clear anything else that might be waiting
                c = Serial.read();
                Serial.println();
                return ret;
            }
        }
        delay(3);
    }
}

void askForSettings() {
    Serial.println("########## Please configure the device ##########");

    Serial.print("Wifi SSID? ");
    config.WifiSSID = readString();
    Serial.print("Wifi Password? ");
    config.WifiPassword = readString();

    Serial.println();

    Serial.print("Device Location? ");
    config.Location = readString();
    Serial.print("Sensor name? ");
    config.Sensor = readString();


    Serial.println();


    Serial.print("Influxdb host? ");
    config.InfluxHost = readString();
    Serial.print("Influxdb database? ");
    config.InfluxDatabase = readString();
    Serial.print("Influxdb username? ");
    config.InfluxUser = readString();
    Serial.print("Influxdb password? ");
    config.InfluxPassword = readString();


    Serial.println();


    Serial.print("Enable Temp Sensor (Y/N)? ");
    config.EnableTempSensor = readBool();
    Serial.print("Enable Wind Sensor (Y/N)? ");
    config.EnableWindSensor = readBool();
    Serial.print("Enable Rain Sensor (Y/N)? ");
    config.EnableRainSensor = readBool();


    Serial.println();

    config.Magic = CONFIG_MAGIC;
    configTool.save();
    Serial.println("Settings saved!");
}

void printBool(bool value) {
    if (value) {
        Serial.println("Yes");
    } else {
        Serial.println("No");
    }
}

void printSettings() {
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
    Serial.println(config.InfluxHost);
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

    Serial.println();
}

void reconfigureCheck() {
    if (Serial.available()) {
        char code = Serial.read();
        if (code == 'i' || code == 'I') {
            printSettings();
            return;
        } else if (code == 'c' || code == 'C') {
            // Reconfigure the sensor
            askForSettings();
        }
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("####### ESP32 Sensor INIT #######");

    // Load Settings if they already exist.
    configTool.addVariable("Magic", &config.Magic);
    configTool.addVariable("WifiSSID", &config.WifiSSID);
    configTool.addVariable("WifiPassword", &config.WifiPassword);

    configTool.addVariable("Location", &config.Location);
    configTool.addVariable("Sensor", &config.Sensor);

    configTool.addVariable("InfluxHost", &config.InfluxHost);
    configTool.addVariable("InfluxDatabase", &config.InfluxDatabase);
    configTool.addVariable("InfluxUser", &config.InfluxUser);
    configTool.addVariable("InfluxPassword", &config.InfluxPassword);

    configTool.addVariable("EnableTempSensor", &config.EnableTempSensor);
    configTool.addVariable("EnableWindSensor", &config.EnableWindSensor);
    configTool.addVariable("EnableRainSensor", &config.EnableRainSensor);

    configTool.load();
    if (config.Magic != CONFIG_MAGIC) {
        askForSettings();
    }

    // wait to see if user wants to update settings
    delay(1000);
    reconfigureCheck();
    Tags = "location=" + config.Location + ",sensor=" + config.Sensor;


    //
    // Set up sensors
    //
    Serial.println("Setting up Sensors...");

    if (config.EnableTempSensor) {
        setupBME280();
    }

    if (config.EnableRainSensor) {
        setupRainSensor();
    }

    if (config.EnableWindSensor) {
        setupWindSensor();
    }


    connectToWifi();

    influx.configure(config.InfluxDatabase.c_str(), config.InfluxHost.c_str());
    influx.authorize(config.InfluxUser.c_str(), config.InfluxPassword.c_str());
    influx.addCertificate(ROOT_CERT);

    Serial.println("Setup Complete. Entering run loop");
}


void loop() {
    if (config.EnableTempSensor && (millis() - tempHumidLastSent) > TEMP_HUMIDITY_DELAY) {
        sendTempAndHumidity();
        tempHumidLastSent = millis();
    }

    if (config.EnableWindSensor && (millis() - windLastSent) > WIND_DELAY) {
        sendWindSpeed();
        windLastSent = millis();
    }

    if (config.EnableRainSensor && (millis() - rainLastSent) > RAIN_DELAY) {
        sendRainfall();
        rainLastSent = millis();
    }

    delay(10);
}
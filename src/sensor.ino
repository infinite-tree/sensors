// Built-in includes
#include <Arduino.h>
#include "Config.h"
#include <driver/adc.h>
#include <WiFi.h>

// To disable wifi chip for sleep mode
#include <esp_wifi.h>

// DNS doesn't seem to work without these
#include "lwip/inet.h"
#include "lwip/dns.h"

#include <Wire.h>
#include <SparkFunBME280.h>

// Downloaded from https://github.com/teebr/Influx-Arduino
#include <InfluxArduino.hpp>
#include "RootCert.hpp"


// Conversions
#define MPS_TO_MPH      2.23694
// Standard cheap tilt rainfall sensor is 0.3mm per tilt, (but it counts 4 times per tilt)
#define COUNT_TO_MM     0.3/4
#define MS_TO_US        1000

// Sensor Pins
#define BME_SDA_PIN     21
#define BME_SDL_PIN     22
#define WIND_PIN        ADC1_CHANNEL_0 // VP/36
#define RAIN_PIN        33
#define RAIN_GPIO       GPIO_NUM_33
#define RAIN_GND        32  // Will need to be physicall connected to gnd
#define SOIL_GND        34
#define SOIL_PIN        ADC1_CHANNEL_3 // VN/39

// How often to send data in seconds
#define TEMP_DELAY_SEC              60
#define WIND_DELAY_SEC              5
#define RAIN_DELAY_SEC              30
#define SOIL_DELAY_SEC              5 * 60
#define SLEEP_DELAY_SEC             5

// How often to send data in milliseconds
#define TEMP_HUMIDITY_DELAY         TEMP_DELAY_SEC * 1000
#define WIND_DELAY                  WIND_DELAY_SEC * 1000
#define RAIN_DELAY                  RAIN_DELAY_SEC * 1000
#define SOIL_DELAY                  SOIL_DELAY_SEC * 1000

#define SLEEP_DELAY                 SLEEP_DELAY_SEC * 1000

// How often to send data in boots
#define TEMP_DELAY_BOOTS            TEMP_DELAY_SEC / SLEEP_DELAY_SEC
#define WIND_DELAY_BOOTS            WIND_DELAY_SEC / SLEEP_DELAY_SEC
#define RAIN_DELAY_BOOTS            RAIN_DELAY_SEC / SLEEP_DELAY_SEC
#define SOIL_DELAY_BOOTS            SOIL_DELAY_SEC / SLEEP_DELAY_SEC

InfluxArduino influx;
BME280 bme;

// rain sensor variables
#define RAIN_SAMPLES                360
RTC_DATA_ATTR volatile uint8_t rainCounter = 0;
RTC_DATA_ATTR uint16_t rainHead = 0;
RTC_DATA_ATTR uint8_t rainSamples[RAIN_SAMPLES] = {0};
RTC_DATA_ATTR portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// Wifi Settings
#define CONNECTION_RETRY        10

// Influx Settings

const char TEMP_MEASUREMENT[] = "temperature_fahrenheit";
const char HUMIDITY_MEASUREMENT[] = "humidity_percentage";
const char PRESSURE_MEASUREMENT[] = "pressure_hpa";
const char WIND_MEASUREMENT[] = "wind_mph";
const char RAIN_MEASUREMENT[] = "rain_mm_hr";
const char SOIL_MEASUREMENT[] = "soil_moisture";

RTC_DATA_ATTR String Tags;
RTC_DATA_ATTR bool firstBoot = true;

// Track the last time data was sent to influx (Non-sleep mode)
unsigned long tempHumidLastSent = 0;
unsigned long windLastSent = 0;
unsigned long rainLastSent = 0;
unsigned long soilLastSent = 0;

// Track the last time data was sent to influx (sleep mode)
RTC_DATA_ATTR unsigned int bootsUntilTemp = 0;
RTC_DATA_ATTR unsigned int bootsUntilWind = 0;
RTC_DATA_ATTR unsigned int bootsUntilRain = 0;
RTC_DATA_ATTR unsigned int bootsUntilSoil = 0;


Config config;

// void printDNSServers()
// {
//     Serial.print("DNS #1, #2 IP: ");
//     WiFi.dnsIP().printTo(Serial);
//     Serial.print(", ");
//     WiFi.dnsIP(1).printTo(Serial);
//     Serial.println();
// }

void connectToWifi()
{
    if (WiFi.status() == WL_CONNECTED)
    {
        Serial.println("Wifi already connceted");
        return;
    }

    WiFi.mode(WIFI_STA);
    while (true)
    {
        Serial.print("Connecting to Wifi network ");
        Serial.print(config.WifiSSID);
        Serial.print(" ");
        WiFi.scanNetworks();
        WiFi.begin(config.WifiSSID.c_str(), config.WifiPassword.c_str());
        for (int x = 0; x < CONNECTION_RETRY; x++)
        {
            if (WiFi.status() == WL_CONNECTED)
            {
                Serial.println("Connected");
                return;
            }
            Serial.print(".");
            delay(500);
        }
    }
}

bool sendDatapoint(const char *measurement, const char *tags, const char *fields) {
    // Make sure there is a connection
    if (WiFi.status() != WL_CONNECTED) {
        connectToWifi();
    }

    // Send the data point
    if (!influx.write(measurement, tags, fields))
    {
        Serial.print("ERROR sending ");
        Serial.print(measurement);
        Serial.print(": ");
        Serial.println(influx.getResponse());
        return false;
    }

    Serial.print("MEASUREMENT: ");
    Serial.print(measurement);
    Serial.print(", tags: ");
    Serial.print(tags);
    Serial.print(", fields: ");
    Serial.println(fields);

    return true;
}

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

    // read 64 times (assuming a 0.1uF cap on the input)
    // https://docs.espressif.com/projects/esp-idf/en/latest/api-reference/peripherals/adc.html#adc-api-adc-calibration
    int16_t sensorSum = 0;
    for (int x=0; x<64; x++) {
        sensorSum += adc1_get_raw(WIND_PIN);
    }

    int16_t sensorValue = sensorSum/64;

    float scaledValue = 0;
    Serial.print("ADC (WIND): ");
    Serial.println(sensorValue);

    // Scale value 0.4V = 0 m/s, 2.0V = 32.4 m/s
    // Measured voltages:
    // 2.0V = 1023 (power supply @2v)
    // 0.4V = 166 (sensor stationary) (250 with battery PSU)
    sensorValue = constrain(sensorValue, 250, 1024);
    scaledValue = mapf(float(sensorValue), 250, 1024, 0.0, 32.4);

    uint16_t value = scaledValue*MPS_TO_MPH;
    Serial.print("W = ");
    Serial.println(value);

    String fields = "value=" + String(value);
    sendDatapoint(WIND_MEASUREMENT, Tags.c_str(), fields.c_str());
}


void setupRainSensor() {
    // Setup Rain sensor
    pinMode(RAIN_GND, OUTPUT);
    digitalWrite(RAIN_GND, LOW);
    pinMode(RAIN_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(RAIN_PIN), rainInterrupt, FALLING);

    esp_sleep_enable_ext0_wakeup(RAIN_GPIO, 0);
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
    float value = sum * COUNT_TO_MM;

    String fields = "value=" + String(value);
    sendDatapoint(RAIN_MEASUREMENT, Tags.c_str(), fields.c_str());
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
    String temp_fields = "value=" + String(temperature);
    String humidity_fields = "value=" + String(humidity);
    // String pressure_fields = "value=" + String(pressure);

    sendDatapoint(TEMP_MEASUREMENT, Tags.c_str(), temp_fields.c_str());
    sendDatapoint(HUMIDITY_MEASUREMENT, Tags.c_str(), humidity_fields.c_str());
    // sendDatapoint(PRESSURE_MEASUREMENT, Tags.c_str(), pressure_fields.c_str());

}

void setupSoilSensor() {
    // Setup soil sensor
    pinMode(SOIL_PIN, INPUT);

    // Disable probe until ready to read tpo reduce corrosion
    pinMode(SOIL_GND, OUTPUT);
    digitalWrite(SOIL_GND, HIGH);

    Serial.println("Soil Sensor initialized");
}

void sendSoilMoisture() {
    digitalWrite(SOIL_GND, LOW);
    // Wait for sensor to become ready
    delay(20);

    // Set the ADC to 10 bits instead of default of 12 (range = 1024)
    adc1_config_width(ADC_WIDTH_BIT_10);
    // Set the attenuation of the Soil sensor pin to 0-3.9V
    adc1_config_channel_atten(SOIL_PIN, ADC_ATTEN_DB_11);
    int16_t sensorValue1 = adc1_get_raw(SOIL_PIN);
    delay(20);
    int16_t sensorValue2 = adc1_get_raw(SOIL_PIN);
    delay(20);
    int16_t sensorValue3 = adc1_get_raw(SOIL_PIN);
    delay(20);

    // disable soil moisture sensor to reduce corrosion
    digitalWrite(SOIL_GND, HIGH);


    int16_t value = (sensorValue1 + sensorValue2 + sensorValue3) / 3;

    // Serial.print("ADC (SOIL): ");
    // Serial.println(sensorValue);

    Serial.print("M = ");
    Serial.println(value);

    String fields = "value=" + String(value);
    sendDatapoint(SOIL_MEASUREMENT, Tags.c_str(), fields.c_str());
}


void reconfigureCheck() {
    if (Serial.available()) {
        char code = Serial.read();
        if (code == 'i' || code == 'I') {
            printConfig(config);
            return;
        } else if (code == 'c' || code == 'C') {
            // Reconfigure the sensor
            askForSettings(config);
            Serial.println("Please reboot now");
            while (1)
            {
            };
        } else if (code == 'p' || code == 'P') {
            askForPreferences(config);
            saveConfig(config);
            Serial.println("Please reboot now");
            while (1)
            {
            };
        }
    }
}

void setupSensors() {
    //
    // Set up sensors
    //
    Serial.println("Setting up Sensors...");

    if (config.EnableTempSensor)
    {
        setupBME280();
    }

    if (config.EnableRainSensor)
    {
        setupRainSensor();
    }

    if (config.EnableWindSensor)
    {
        setupWindSensor();
    }

    if (config.EnableSoilSensor)
    {
        setupSoilSensor();
    }
}

void setupInflux() {
    influx.configure(config.InfluxDatabase.c_str(), config.InfluxHostname.c_str());
    influx.authorize(config.InfluxUser.c_str(), config.InfluxPassword.c_str());
    influx.addCertificate(ROOT_CERT);
}

void SleepModeSetup() {
    // TODO: confirm this bug is real. Workaround for reading from
    //       RTC memory: https://www.hackster.io/nickthegreek82/esp32-deep-sleep-tutorial-4398a7
    delay(500);
    Tags = "location=" + config.Location + ",sensor=" + config.Sensor;

    // First boot only
    if (firstBoot)
    {
        firstBoot = false;

        // wait to see if user wants to update settings
        delay(500);
        reconfigureCheck();

        Serial.println();
        Serial.print("Temp delay boots: ");
        Serial.println(TEMP_DELAY_BOOTS);
        Serial.print("Rain delay boots: ");
        Serial.println(RAIN_DELAY_BOOTS);
        Serial.print("Wind delay boots: ");
        Serial.println(WIND_DELAY_BOOTS);
        Serial.print("Soil delay boots: ");
        Serial.println(SOIL_DELAY_BOOTS);
        Serial.println();
    }

    Serial.println("Sleep Mode Setup Complete. Entering run loop");
}

void sleepModeMain() {
    esp_sleep_wakeup_cause_t wakeup_reason;
    wakeup_reason = esp_sleep_get_wakeup_cause();
    // If the device was woken due to external interrupt, process it and go back to sleep
    if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT0)
    {
        if (config.EnableRainSensor) {
            setupRainSensor();
            portENTER_CRITICAL_ISR(&mux);
            rainCounter++;
            portEXIT_CRITICAL_ISR(&mux);
        }
        esp_sleep_enable_timer_wakeup(SLEEP_DELAY * MS_TO_US);
        esp_wifi_stop();
        esp_deep_sleep_start();
    }

    // Evaluate Rain sensor first since it can be woken by an external interrupt
    if (config.EnableRainSensor)
    {
        //  rain interrupts asap
        setupRainSensor();

        Serial.print("Boots until Rain data: ");
        Serial.println(bootsUntilRain);
        if (bootsUntilRain == 0)
        {
            bootsUntilRain = RAIN_DELAY_BOOTS - 1;

            connectToWifi();
            setupInflux();
            Serial.println("Send rain ...");
            sendRainfall();
        }
        else
        {
            bootsUntilRain--;
        }
    }

    // determine which sensor to read and then how
    // long to sleep for
    if (config.EnableTempSensor) {
        Serial.print("Boots until Temp data: ");
        Serial.println(bootsUntilTemp);
        if (bootsUntilTemp == 0) {
            bootsUntilTemp = TEMP_DELAY_BOOTS - 1;

            connectToWifi();
            setupInflux();

            setupBME280();
            sendTempAndHumidity();
        } else {
            bootsUntilTemp--;
        }
    }

    if (config.EnableWindSensor) {
        Serial.print("Boots until Wind data: ");
        Serial.println(bootsUntilWind);
        if (bootsUntilWind == 0)
        {
            bootsUntilWind = WIND_DELAY_BOOTS - 1;
            connectToWifi();
            setupInflux();

            setupWindSensor();
            sendWindSpeed();
        } else {
            bootsUntilWind--;
        }
    }

    if (config.EnableSoilSensor) {
        Serial.print("Boots until Soil data: ");
        Serial.println(bootsUntilSoil);
        if (bootsUntilSoil == 0)
        {
            bootsUntilSoil = SOIL_DELAY_BOOTS - 1;
            connectToWifi();
            setupInflux();

            setupSoilSensor();
            sendSoilMoisture();
        } else {
            bootsUntilWind--;
        }
    }


    // Deep Sleep
    esp_sleep_enable_timer_wakeup(SLEEP_DELAY * MS_TO_US);
    esp_wifi_stop();
    esp_deep_sleep_start();
}

void setup() {
    Serial.begin(115200);
    Serial.println("####### ESP32 Sensor INIT #######");

    loadConfig(config);
    if (config.Magic != CONFIG_MAGIC) {
        askForSettings(config);
    }

    // Sleep Mode vs normal mode follow different models.
    // Under sleep mode, the device is effectively reset every wake up
    // and starts with the setup code each time
    if (config.EnableSleepMode) {
        SleepModeSetup();
        // does not return
        sleepModeMain();
    }

    //
    //  Normal Setup
    //

    // wait to see if user wants to update settings
    delay(1000);
    reconfigureCheck();
    Tags = "location=" + config.Location + ",sensor=" + config.Sensor;

    setupSensors();
    connectToWifi();
    setupInflux();
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

    if (config.EnableSoilSensor && (millis() - soilLastSent) > SOIL_DELAY) {
        sendSoilMoisture();
        soilLastSent = millis();
    }

    delay(10);
}
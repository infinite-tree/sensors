/*
Deep Sleep with External Wake Up
=====================================
This code displays how to use deep sleep with
an external trigger as a wake up source and how
to store data in RTC memory to use it over reboots
This code is under Public Domain License.
Hardware Connections
======================
Push Button to GPIO 33 pulled down with a 10K Ohm
resistor
NOTE:
======
Only RTC IO can be used as a source for external wake
source. They are pins: 0,2,4,12-15,25-27,32-39.
Author:
Pranav Cherukupalli <cherukupallip@gmail.com>
*/

#include <Arduino.h>

#define BUTTON_PIN_BITMASK 0x200000000 // 2^33 in hex
#define RAIN_PIN 33
#define RAIN_GND 32

RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR int rainCounter = 0;
RTC_DATA_ATTR portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;


void setupRainSensor()
{
  // Setup Rain sensor
  pinMode(RAIN_GND, OUTPUT);
  digitalWrite(RAIN_GND, LOW);
  pinMode(RAIN_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RAIN_PIN), rainInterrupt, FALLING);
  Serial.println("Rain sensor initialized");
}

void IRAM_ATTR rainInterrupt()
{
  portENTER_CRITICAL_ISR(&mux);
  rainCounter++;
  portEXIT_CRITICAL_ISR(&mux);
}


/*
Method to print the reason by which ESP32
has been awaken from sleep
*/
void print_wakeup_reason()
{
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason)
  {
  case ESP_SLEEP_WAKEUP_EXT0:
    Serial.println("Wakeup caused by external signal using RTC_IO");
    break;
  case ESP_SLEEP_WAKEUP_EXT1:
    Serial.println("Wakeup caused by external signal using RTC_CNTL");
    break;
  case ESP_SLEEP_WAKEUP_TIMER:
    Serial.println("Wakeup caused by timer");
    break;
  case ESP_SLEEP_WAKEUP_TOUCHPAD:
    Serial.println("Wakeup caused by touchpad");
    break;
  case ESP_SLEEP_WAKEUP_ULP:
    Serial.println("Wakeup caused by ULP program");
    break;
  default:
    Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason);
    break;
  }
}

void setup()
{
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  Serial.begin(115200);

  setupRainSensor();


  //Increment boot number and print it every reboot
  ++bootCount;
  Serial.println("Boot number: " + String(bootCount));

  portENTER_CRITICAL_ISR(&mux);
  Serial.println("Rain counter: " +  String(rainCounter));
  portEXIT_CRITICAL_ISR(&mux);

  //Print the wakeup reason for ESP32
  print_wakeup_reason();

  /*
  First we configure the wake up source
  We set our ESP32 to wake up for an external trigger.
  There are two types for ESP32, ext0 and ext1 .
  ext0 uses RTC_IO to wakeup thus requires RTC peripherals
  to be on while ext1 uses RTC Controller so doesnt need
  peripherals to be powered on.
  Note that using internal pullups/pulldowns also requires
  RTC peripherals to be turned on.
  */
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_33, 0); //1 = High, 0 = Low

  //If you were to use ext1, you would use it like
  //esp_sleep_enable_ext1_wakeup(BUTTON_PIN_BITMASK,ESP_EXT1_WAKEUP_ANY_HIGH);

  //Go to sleep now
  Serial.println("Going to sleep now");
  esp_deep_sleep_start();
  Serial.println("This will never be printed");
}

void loop()
{
  //This is not going to be called
}
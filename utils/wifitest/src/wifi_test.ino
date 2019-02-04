#include <WiFi.h>


String translateEncryptionType(wifi_auth_mode_t encryptionType) {


  switch (encryptionType) {
    case (WIFI_AUTH_OPEN):
      return "Open";
    case (WIFI_AUTH_WEP):
      return "WEP";
    case (WIFI_AUTH_WPA_PSK):
      return "WPA_PSK";
    case (WIFI_AUTH_WPA2_PSK):
      return "WPA2_PSK";
    case (WIFI_AUTH_WPA_WPA2_PSK):
      return "WPA_WPA2_PSK";
    case (WIFI_AUTH_WPA2_ENTERPRISE):
      return "WPA2_ENTERPRISE";
  }
}

void scanNetworks() {

  int numberOfNetworks = WiFi.scanNetworks();

  Serial.print("Number of networks found: ");
  Serial.println(numberOfNetworks);

  for (int i = 0; i < numberOfNetworks; i++) {

    Serial.print("Network name: ");
    Serial.println(WiFi.SSID(i));

    Serial.print("Signal strength: ");
    Serial.println(WiFi.RSSI(i));

    Serial.print("MAC address: ");
    Serial.println(WiFi.BSSIDstr(i));

    Serial.print("Encryption type: ");
    String encryptionTypeDescription = translateEncryptionType(WiFi.encryptionType(i));
    Serial.println(encryptionTypeDescription);
    Serial.println("-----------------------");

  }
}

void connectToNetwork(String ssid, String password) {
  WiFi.begin(ssid.c_str(), password.c_str());

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Establishing connection to WiFi..");
  }

  Serial.println("Connected to network");

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

void setup() {

  Serial.begin(115200);
  Serial.print("Enter your Wifi SSID: ");
  String ssid = readString();
  Serial.print("Enter the password: ");
  String password = readString();


  scanNetworks();
  connectToNetwork(ssid, password);

  Serial.println(WiFi.macAddress());
  Serial.println(WiFi.localIP());

  WiFi.disconnect(true);
  Serial.println(WiFi.localIP());

}

void loop() {}

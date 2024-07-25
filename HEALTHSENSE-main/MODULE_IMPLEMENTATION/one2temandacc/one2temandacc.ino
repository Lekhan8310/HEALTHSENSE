#include <BTAddress.h>
#include <BTAdvertisedDevice.h>
#include <BTScan.h>
#include <BluetoothSerial.h>

#include <DHT.h>
#include <Arduino.h>
#if defined(ESP32)
  #include <WiFi.h>
#elif defined(ESP8266)
  #include <ESP8266WiFi.h>
#endif
#include <Firebase_ESP_Client.h>
#include <Wire.h>

// WiFi and ThingSpeak parameters
//Provide the token generation process info.
#include "addons/TokenHelper.h"
//Provide the RTDB payload printing info and other helper functions.
#include "addons/RTDBHelper.h"

#define USE_NAME // Comment this to use MAC address instead of a slaveName
const char *pin = "1234"; // Change this to reflect the pin expected by the real slave BT device

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

BluetoothSerial SerialBT;

#ifdef USE_NAME
  String slaveName = "ESP32-BT-Slave"; // Change this to reflect the real name of your slave BT device
#else
  String MACadd = "A0:A3:B3:AB:69:08"; // This only for printing
  uint8_t address[6]  = {0xA0, 0xA3, 0xB3, 0xAB, 0x69, 0x08}; // Change this to reflect real MAC address of your slave BT device
#endif

String myName = "ESP32-BT-Master";

// Insert your network credentials
#define WIFI_SSID "Azmal"
#define WIFI_PASSWORD "King$123"
// Insert Firebase project API Key
#define API_KEY "AIzaSyBbklMa8cmYBN7obtTjd46OIxq7YbAnv-k"

// Insert RTDB URLefine the RTDB URL */
#define DATABASE_URL "https://healthsense-8741d-default-rtdb.firebaseio.com/" 

//Define Firebase Data object
FirebaseData fbdo;

FirebaseAuth auth;
FirebaseConfig config;
unsigned long sendDataPrevMillis = 0;
int count = 0;
bool signupOK = false;
int thresholdRSSI = -70;
// DHT sensor pin
#define DHTPIN 13  // Example pin

// Define DHT type (DHT11)
#define DHTTYPE DHT11

// Initialize DHT sensor
DHT dht(DHTPIN, DHTTYPE);
WiFiClient client;

void setup() {
  bool connected;
  Serial.begin(115200);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED){
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  /* Assign the api key (required) */
  config.api_key = API_KEY;

  /* Assign the RTDB URL (required) */
  config.database_url = DATABASE_URL;
  if (Firebase.signUp(&config, &auth, "", "")){
    Serial.println("ok");
    signupOK = true;
  }
  else{
    Serial.printf("%s\n", config.signer.signupError.message.c_str());
  }

  /* Assign the callback function for the long running token generation task */
  config.token_status_callback = tokenStatusCallback; //see addons/TokenHelper.h
  
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
  SerialBT.begin(myName, true);
  Serial.printf("The device \"%s\" started in master mode, make sure slave BT device is on!\n", myName.c_str());

  #ifndef USE_NAME
    SerialBT.setPin(pin);
    Serial.println("Using PIN");
  #endif

  // connect(address) is fast (up to 10 secs max), connect(slaveName) is slow (up to 30 secs max) as it needs
  // to resolve slaveName to address first, but it allows to connect to different devices with the same name.
  // Set CoreDebugLevel to Info to view devices Bluetooth address and device names
  #ifdef USE_NAME
    connected = SerialBT.connect(slaveName);
    Serial.printf("Connecting to slave BT device named \"%s\"\n", slaveName.c_str());
  #else
    connected = SerialBT.connect(address);
    Serial.print("Connecting to slave BT device with MAC "); Serial.println(MACadd);
  #endif

  if(connected) {
    Serial.println("Connected Successfully!");
  } else {
    while(!SerialBT.connected(10000)) {
      Serial.println("Failed to connect. Make sure remote device is available and in range, then restart app.");
    }
  }
  // Disconnect() may take up to 10 secs max
  if (SerialBT.disconnect()) {
    Serial.println("Disconnected Successfully!");
  }
  // This would reconnect to the slaveName(will use address, if resolved) or address used with connect(slaveName/address).
  SerialBT.connect();
  if(connected) {
    Serial.println("Reconnected Successfully!");
  } else {
    while(!SerialBT.connected(10000)) {
      Serial.println("Failed to reconnect. Make sure remote device is available and in range, then restart app.");
    }
  }
}

void loop() {
   if (SerialBT.available()) {
    float acceleration = SerialBT.parseFloat(); // Read acceleration data from One1

    // Calculate distance based on acceleration (example calculation)
    float distance = 0.5 * acceleration; // Example formula, adjust as needed

    Serial.print("Distance Traveled: ");
    Serial.println(distance);
      delay(1000); // Adjust delay based on your requirements
  }

  // Read temperature and humidity from DHT sensor
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();

  // Print data to serial monitora
  Serial.print("Temperature: ");
  Serial.println(temperature);
  Serial.print("Humidity: ");
  Serial.println(humidity);
  if (SerialBT.available()) {
    float acceleration = SerialBT.parseFloat(); // Read the incoming acceleration data from One1
    Serial.println(acceleration);
     if(Firebase.RTDB.setFloat(&fbdo,"movement/acceleration", acceleration)){
      Serial.println("PASSED");
      Serial.println("TYPE: " + fbdo.dataType());
    }
  }
    // Check if the acceleration is greater than 10
      // Perform actions or send commands to One1 or other devices as needed
  if (Firebase.ready() && signupOK && (millis() - sendDataPrevMillis > 1000 || sendDataPrevMillis == 0)){
    sendDataPrevMillis = millis();
    // Write an Int number on the database path test/int
    if (Firebase.RTDB.setFloat(&fbdo,"test/Temperature",temperature)){
      Serial.println("PASSED");
      Serial.println("TYPE: " + fbdo.dataType());
    }
    else {
      Serial.println("FAILED");
      Serial.println("REASON: " + fbdo.errorReason());
    }
     if (Firebase.RTDB.setFloat(&fbdo,"test/Humidity",humidity)){
      Serial.println("PASSED");
      Serial.println("TYPE: " + fbdo.dataType());
    }
    else {
      Serial.println("FAILED");
      Serial.println("REASON: " + fbdo.errorReason());
    }
}
}

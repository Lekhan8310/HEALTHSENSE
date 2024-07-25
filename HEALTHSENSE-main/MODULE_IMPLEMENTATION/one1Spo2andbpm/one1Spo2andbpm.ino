// Tinker Foundry
// Accompanying video: https://youtu.be/xjwSKy6jzTI
// Code for ESP32 to interface with MAX30102 breakout board and report blood oxygen level
// Refer to https://github.com/DKARDU/bloodoxygen - below code is simplified version
// https://www.analog.com/media/en/technical-documentation/data-sheets/MAX30102.pdf
// Connections from WEMOS D1 R32 board to MAX30102 Breakout Board as follows:
//  SCL (ESP32) to SCL (breakout board)
//  SDA (ESP32) to SDA (breakout board)
//  3V3 (ESP32) to VIN (breakout board)
//  GND (ESP32) to GND (breakout board)
#include <Arduino.h>
#if defined(ESP32)
  #include <WiFi.h>
#elif defined(ESP8266)
  #include <ESP8266WiFi.h>
#endif
#include <Firebase_ESP_Client.h>
#include <Wire.h>
#include "MAX30105.h" //sparkfun MAX3010X library
MAX30105 particleSensor;
#include "BluetoothSerial.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
//#define USE_PIN // Uncomment this to use PIN during pairing. The pin is specified on the line below
const char *pin = "1234"; // Change this to more secure PIN.

String device_name = "ESP32-BT-Slave";
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

BluetoothSerial SerialBT;

//Provide the token generation process info.
#include "addons/TokenHelper.h"
//Provide the RTDB payload printing info and other helper functions.
#include "addons/RTDBHelper.h"

// Insert your network credentials
#define WIFI_SSID "Azmal"
#define WIFI_PASSWORD "King$1234"

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
double avered    = 0; 
double aveir     = 0;
double sumirrms  = 0;
double sumredrms = 0;
int    i         = 0;
int    Num       = 100;  // calculate SpO2 by this sampling interval
int    Temperature;
int    temp;
float  ESpO2;            // initial value of estimated SpO2
double FSpO2     = 0.7;  // filter factor for estimated SpO2
double frate     = 0.95; // low pass filter for IR/red LED value to eliminate AC component
#define TIMETOBOOT 3000  // wait for this time(msec) to output SpO2
#define SCALE      88.0  // adjust to display heart beat and SpO2 in the same scale
#define SAMPLING   100 //25 //5     // if you want to see heart beat more precisely, set SAMPLING to 1
#define FINGER_ON  30000 // if red signal is lower than this, it indicates your finger is not on the sensor
#define USEFIFO


void setup()
{
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
  // Initialize sensor
  while (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30102 was not found. Please check wiring/power/solder jumper at MH-ET LIVE MAX30102 board. ");
    //while (1);
  }
  SerialBT.begin(device_name); //Bluetooth device name
  Serial.printf("The device with name \"%s\" is started.\nNow you can pair it with Bluetooth!\n", device_name.c_str());
  //Serial.printf("The device with name \"%s\" and MAC address %s is started.\nNow you can pair it with Bluetooth!\n", device_name.c_str(), SerialBT.getMacString()); // Use this after the MAC method is implemented
  #ifdef USE_PIN
    SerialBT.setPin(pin);
    Serial.println("Using PIN");
  #endif
   if (!accel.begin()) {
    Serial.println("Could not find a valid ADXL345 sensor, check wiring!");
    while (1);
  }
   accel.setRange(ADXL345_RANGE_16_G);

  //Setup to sense a nice looking saw tooth on the plotter
  byte ledBrightness = 0x7F; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode       = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  //Options: 1 = IR only, 2 = Red + IR on MH-ET LIVE MAX30102 board
  int sampleRate     = 200; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth     = 411; //Options: 69, 118, 215, 411
  int adcRange       = 16384; //Options: 2048, 4096, 8192, 16384
  
  // Set up the wanted parameters
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings

  particleSensor.enableDIETEMPRDY();
}

void loop()
{

  uint32_t ir, red, green;
  double fred, fir;
  double SpO2 = 0; //raw SpO2 before low pass filtered
  
#ifdef USEFIFO
  particleSensor.check(); //Check the sensor, read up to 3 samples

  while (particleSensor.available()) {//do we have new data
#ifdef MAX30105
   red = particleSensor.getFIFORed(); //Sparkfun's MAX30105
   ir  = particleSensor.getFIFOIR();  //Sparkfun's MAX30105
#else
   red = particleSensor.getFIFOIR();  //why getFOFOIR output Red data by MAX30102 on MH-ET LIVE breakout board
   ir  = particleSensor.getFIFORed(); //why getFIFORed output IR data by MAX30102 on MH-ET LIVE breakout board
#endif
   
    i++;
    fred = (double)red;
    fir  = (double)ir;
    avered = avered * frate + (double)red * (1.0 - frate); //average red level by low pass filter
    aveir = aveir * frate + (double)ir * (1.0 - frate); //average IR level by low pass filter
    sumredrms += (fred - avered) * (fred - avered); //square sum of alternate component of red level
    sumirrms += (fir - aveir) * (fir - aveir);//square sum of alternate component of IR level
    if ((i % SAMPLING) == 0) {//slow down graph plotting speed for arduino Serial plotter by thin out
      if ( millis() > TIMETOBOOT) {
        float ir_forGraph = (2.0 * fir - aveir) / aveir * SCALE;
        float red_forGraph = (2.0 * fred - avered) / avered * SCALE;
        //truncation for Serial plotter's autoscaling
        if ( ir_forGraph > 100.0) ir_forGraph = 100.0;
        if ( ir_forGraph < 80.0) ir_forGraph = 80.0;
        if ( red_forGraph > 100.0 ) red_forGraph = 100.0;
        if ( red_forGraph < 80.0 ) red_forGraph = 80.0;
        // Print out red and IR sensor reading to serial interface for monitoring...
        Serial.print("Red: "); Serial.print(red); Serial.print(","); Serial.print("Infrared: "); Serial.print(ir); Serial.print(".    ");
        float temperature = particleSensor.readTemperatureF();
        
        if (ir < FINGER_ON){ // no finger on the sensor
           Serial.println("No finger detected");
           ESpO2=0;
           Firebase.RTDB.setFloat(&fbdo,"PatientHealth/SpO2",ESpO2);
           Firebase.RTDB.setString(&fbdo, "PatientHealth/Btprox", "Far");
           break;
        }
        if(ir > FINGER_ON){
           //Temperature = mlx.readObjectTempC();
            Serial.println("Finger detected");
           Firebase.RTDB.setString(&fbdo, "PatientHealth/Btprox", "Near");
           Serial.print("Oxygen % = ");
           Serial.print(ESpO2);
           Serial.println("%");
           Firebase.RTDB.setInt(&fbdo,"PatientHealth/SpO2",ESpO2);
           
        }
      }
    }
    if ((i % Num) == 0) {
      double R = (sqrt(sumredrms) / avered) / (sqrt(sumirrms) / aveir);
      //Serial.println(R);
      SpO2 = -23.3 * (R - 0.4) + 100; //http://ww1.microchip.com/downloads/jp/AppNotes/00001525B_JP.pdf -- I don't see this directly in the App Note... look here https://github.com/espressif/arduino-esp32/issues/4561
      ESpO2 = FSpO2 * ESpO2 + (1.0 - FSpO2) * SpO2;//low pass filter
      // Serial.print(SpO2);Serial.print(",");Serial.println(ESpO2);
      sumredrms = 0.0; sumirrms = 0.0; i = 0;
      break;
    }

#endif
  particleSensor.nextSample(); //We're finished with this sample so move to next sample
   // Serial.println(SpO2);
  }
  sensors_event_t event;
  accel.getEvent(&event);

  float acceleration = sqrt(event.acceleration.x * event.acceleration.x +
                            event.acceleration.y * event.acceleration.y +
                            event.acceleration.z * event.acceleration.z);
  SerialBT.print(acceleration);
  SerialBT.println();
}
#include <Arduino.h>
#include <Stepper.h>
#include <time.h>
#if defined(ESP32)
#include <WiFi.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#endif
#include <Firebase_ESP_Client.h>

//Provide the token generation process info.
#include "addons/TokenHelper.h"
//Provide the RTDB payload printing info and other helper functions.
#include "addons/RTDBHelper.h"

// Insert your network credentials
//#define WIFI_SSID "HTWWWZULU-5G"
//#define WIFI_PASSWORD "P0reAdel"

// #define WIFI_SSID "MyAltice 31d611"
// #define WIFI_PASSWORD "3731-gold-55"

//Provide the token generation process info.
#include "addons/TokenHelper.h"
//Provide the RTDB payload printing info and other helper functions.
#include "addons/RTDBHelper.h"
#include "esp_wpa2.h" //wpa2 library for connections to Enterprise networks
#define EAP_IDENTITY "samueldixon@tamu.edu" //if connecting from another corporation, use identity@organisation.domain in Eduroam 
#define EAP_USERNAME "samueldixon@tamu.edu" //oftentimes just a repeat of the identity
#define EAP_PASSWORD "Keeponrunning2021!" //your Eduroam password

// Insert Firebase project API Key
#define API_KEY "AIzaSyCE-ZxzhjvX3JlLi0XG0UbOObWI3Hsi6fg"

// Insert RTDB URLefine the RTDB URL */
#define DATABASE_URL "https://capstone-database-c7175-default-rtdb.firebaseio.com/"

//Define Firebase Data object
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

const int stepsPerRevolution = 800;  // change this to fit the number of steps per revolution

// ULN2003 Motor Driver Pins
#define IN1 19
#define IN2 18
#define IN3 5
#define IN4 17

// initialize the stepper library
Stepper myStepper(stepsPerRevolution, IN1, IN3, IN2, IN4); // instatiate the stepper motor based off the pins defined

unsigned long sendDataPrevMillis = 0; 
int count = 0;
bool signupOK = false; // to make sure wifi works
const char* ssid = "eduroam"; // Eduroam SSID
const char* host = "arduino.php5.sk"; //external server domain for HTTP connection after authentification

// Getting time from NTP Server
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = -21600; // calculate the corresponding offsets 
const int daylightOffset_sec = 3600; // for the timezone of college station
struct tm timeinfo;
char timeStringBuff[50]; //50 chars should be enough

void update_time(String Sensor) { // update time utilizes time.h library to pull in the time to update the sensor readings
  getLocalTime(&timeinfo); // get the local time and store into the reference of timeinfo
  strftime(timeStringBuff, sizeof(timeStringBuff), "%A, %B %d %Y %H:%M:%S", &timeinfo); // storing the time reference infromation into the char[] buffer
  Firebase.RTDB.setString(&fbdo, "Sensors/"+Sensor+"/reading/timestamp", timeStringBuff); // setting the relevant sensor data to the timestringbuff
}

void setup() {
  Serial.begin(115200); // begin the serial terimnal with baud rate of 115200
  WiFi.mode(WIFI_STA); // set the wifi mode to begin
   WiFi.begin(ssid, WPA2_AUTH_PEAP, EAP_IDENTITY, EAP_USERNAME, EAP_PASSWORD); // connect to wifi with WPA enterpirse parameters. 
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) { // while not connected delay 
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  /* Assign the api key (required) */
  config.api_key = API_KEY; // configurating the database api key

  /* Assign the RTDB URL (required) */
  config.database_url = DATABASE_URL; // configurating the database url

  /* Sign up */
  if (Firebase.signUp(&config, &auth, "", "")) {
    Serial.println("ok");
    signupOK = true; // signup varified by method above, so verify and update boolean
  } else {
    Serial.printf("%s\n", config.signer.signupError.message.c_str());
  }

  /* Assign the callback function for the long running token generation task */
  config.token_status_callback = tokenStatusCallback;  //see addons/TokenHelper.h

  Firebase.begin(&config, &auth); // begin the firebase 
  Firebase.reconnectWiFi(true); // reconnect to the wifi (if possible)

  // set the speed at 60 rpm:
  myStepper.setSpeed(10); // set the speed of the mottor
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer); // configure the time from the ntp server
  
}

void loop() {
  if (Firebase.RTDB.getInt(&fbdo, "Flags/Connect")) { // make sure that the connection flag has been set
    if (fbdo.intData() == 1) {  // if the reference is 1, then we can do movement
      if (Firebase.RTDB.getInt(&fbdo, "Flags/Right")) { // now accessing the firebase database object and setting the right flag
        if (fbdo.intData() == 1) { // if the value of the flg is 1
          
          Serial.println("right == 1 , clockwise"); // print serial data and note direction of motor
          
          Firebase.RTDB.setInt(&fbdo, "Flags/Right", 0); // reset the right flag to be 0 
          
          myStepper.step(stepsPerRevolution); // use stepper module to actuate the movement of the motor
          
          Firebase.RTDB.setFloat(&fbdo, "Sensors/Magnetometer1/reading/data", random(1,100)); // testing a random datapoint to be set to the database
          update_time("Magnetometer1"); // call subroutine to update the time of the magnetometer1 sensor

          Firebase.RTDB.setFloat(&fbdo, "Sensors/Magnetometer2/reading/data", random(1,100)); // testing a random datapoint to be set to the database
          update_time("Magnetometer2"); // call subroutine to update the time of the magnetometer2 sensor

          Firebase.RTDB.setFloat(&fbdo, "Sensors/PhotoDiode/reading/data", random(1,100)); // testing a random datapoint to be set to the database
          update_time("PhotoDiode"); // call subroutine to update the time of the photodiode sensor
        
          Firebase.RTDB.setInt(&fbdo, "Flags/Right", 0);  // otherwise print 0 to indicate no movement

        } else {
          Serial.println("right == 0, no movement"); // otherwise we do not move and serial print to the screen
          delay(500); // delay 500 ms so that we can see what is happening logically
        }
      } else {
        Serial.println("FAILED"); // otherwise print that there was a failure
        Serial.println("REASON: " + fbdo.errorReason()); // serially print the reason for error
      }

      if (Firebase.RTDB.getInt(&fbdo, "Flags/Left")) { // make sure that the connection flag has been set
        if (fbdo.intData() == 1) { // if the reference is 1, then we can do movement
          Serial.println("left == 1 , counterclockwise");
          myStepper.step(-stepsPerRevolution);
          Firebase.RTDB.setInt(&fbdo, "Flags/Left", 0); // reset the left flag to 0
        } else {
          Serial.println("left == 0, no movement");  // otherwise print 0 to indicate no movement
          delay(500); // delay by 500 ms
        }
      } else {
        Serial.println("FAILED");
        Serial.println("REASON: " + fbdo.errorReason());
      }
    }
  }
}

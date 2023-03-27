#include <Arduino.h>

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
#include "esp_wpa2.h" //wpa2 library for connections to Enterprise networks

#define EAP_IDENTITY "samueldixon@tamu.edu" //if connecting from another corporation, use identity@organisation.domain in Eduroam 
#define EAP_PASSWORD "Keeponrunning2021!" //your Eduroam password

// Insert Firebase project API Key
#define API_KEY "AIzaSyCE-ZxzhjvX3JlLi0XG0UbOObWI3Hsi6fg"

// Insert RTDB URLefine the RTDB URL */
#define DATABASE_URL "https://capstone-database-c7175-default-rtdb.firebaseio.com/" 

//Define Firebase Data object
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

unsigned long sendDataPrevMillis = 0;
int count = 0;
bool signupOK = false;
const char* ssid = "eduroam"; // Eduroam SSID
const char* host = "arduino.php5.sk"; //external server domain for HTTP connection after authentification
int counter = 0;

// NOTE: For some systems, various certification keys are required to connect to the wifi system.
//       Usually you are provided these by the IT department of your organization when certs are required
//       and you can't connect with just an identity and password.
//       Most eduroam setups we have seen do not require this level of authentication, but you should contact
//       your IT department to verify.
//       You should uncomment these and populate with the contents of the files if this is required for your scenario (See Example 2 and Example 3 below).
//const char *ca_pem = "insert your CA cert from your .pem file here";
//const char *client_cert = "insert your client cert from your .crt file here";
//const char *client_key = "insert your client key from your .key file here";

void setup() {
  Serial.begin(115200);
  delay(10);
  Serial.println();
  Serial.print("Connecting to network: ");
  Serial.println(ssid);
  WiFi.disconnect(true);  //disconnect form wifi to set new wifi connection
  WiFi.mode(WIFI_STA); //init wifi mode
  
  // Example1 (most common): a cert-file-free eduroam with PEAP (or TTLS)
  WiFi.begin(ssid, WPA2_AUTH_PEAP, EAP_IDENTITY, EAP_USERNAME, EAP_PASSWORD);

  // Example 2: a cert-file WPA2 Enterprise with PEAP
  //WiFi.begin(ssid, WPA2_AUTH_PEAP, EAP_IDENTITY, EAP_USERNAME, EAP_PASSWORD, ca_pem, client_cert, client_key);
  
  // Example 3: TLS with cert-files and no password
  //WiFi.begin(ssid, WPA2_AUTH_TLS, EAP_IDENTITY, NULL, NULL, ca_pem, client_cert, client_key);
  
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    counter++;
    if(counter>=60){ //after 30 seconds timeout - reset board
      ESP.restart();
    }
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address set: "); 
  Serial.println(WiFi.localIP()); //print LAN IP

   config.api_key = API_KEY;

  /* Assign the RTDB URL (required) */
  config.database_url = DATABASE_URL;

  /* Sign up */
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

}
void loop() {
  if (WiFi.status() == WL_CONNECTED) { //if we are connected to Eduroam network
    counter = 0; //reset counter
    Serial.println("Wifi is still connected with IP: "); 
    Serial.println(WiFi.localIP());   //inform user about his IP address
  }else if (WiFi.status() != WL_CONNECTED) { //if we lost connection, retry
    WiFi.begin(ssid);      
  }
  while (WiFi.status() != WL_CONNECTED) { //during lost connection, print dots
    delay(500);
    Serial.print(".");
    counter++;
    if(counter>=60){ //30 seconds timeout - reset board
    ESP.restart();
    }
  }
  Serial.print("Connecting to website: ");
  Serial.println(host);
  WiFiClient client;
  if (client.connect(host, 80)) {
    String url = "/rele/rele1.txt";
    client.print(String("GET ") + url + " HTTP/1.1\r\n" + "Host: " + host + "\r\n" + "User-Agent: ESP32\r\n" + "Connection: close\r\n\r\n");

    while (client.connected()) {
      Serial.print("Connected to the datbase and ready to go");
  //     String line = client.readStringUntil('\n');
  //     if (line == "\r") {
  //       break;
  //     }
  //   }
  //   String line = client.readStringUntil('\n');
  //  Serial.println(line);
  // }else{
  //     Serial.println("Connection unsucessful");
  //   }

  //     if (Firebase.ready() && signupOK && (millis() - sendDataPrevMillis > 15000 || sendDataPrevMillis == 0)){
  //   sendDataPrevMillis = millis();

  //   // Write an Int number on the database path test/int
  //   if (Firebase.RTDB.setString(&fbdo, "Sensors/Magnetometer1/reading/timestamp", "hi")){
  //     Serial.println("PASSED");
  //     Serial.println("PATH: " + fbdo.dataPath());
  //     Serial.println("TYPE: " + fbdo.dataType());
  //   }
  //   else {
  //     Serial.println("FAILED");
  //     Serial.println("REASON: " + fbdo.errorReason());
  //   }
  //   count++;
    
  //   // Write an Float number on the database path test/float
  //   if (Firebase.RTDB.setString(&fbdo, "Sensors/Magnetometer1/reading/timestamp", "success")){
  //     Serial.println("PASSED");
  //     Serial.println("PATH: " + fbdo.dataPath());
  //     Serial.println("TYPE: " + fbdo.dataType());
  //   }
  //   else {
  //     Serial.println("FAILED");
  //     Serial.println("REASON: " + fbdo.errorReason());
  //   }
  // }
  
}

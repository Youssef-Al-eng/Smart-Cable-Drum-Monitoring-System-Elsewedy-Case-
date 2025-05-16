#include <WiFi.h>
#include <HardwareSerial.h>
#include <Firebase_ESP_Client.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"
#include "time.h"
#define WIFI_SSID "nour"
#define WIFI_PASSWORD "11111111"
#define API_KEY "AIzaSyBQNhr0YPTm4YciRDqeOAQHmGb1rSnjpoM"
#define DATABASE_URL "https://electronic-systems-in-action-default-rtdb.firebaseio.com"
#define RX_PIN 16
#define TX_PIN 17
const char* ntpServer = "pool.ntp.org";
const long utcOffsetInSeconds = 2 * 3600;
HardwareSerial mySerial(1);

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
int idx = 0;
const int numValues = 5;
double values[numValues];

void setup() {
  Serial.begin(115200);
  mySerial.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();
  configTime(utcOffsetInSeconds, 0, ntpServer);
  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;
  if (Firebase.signUp(&config, &auth, "", "")) {
    Serial.println("Firebase sign-up succeeded");
  } else {
    Serial.printf("Firebase sign-up error: %s\n", config.signer.signupError.message.c_str());
  }
  config.token_status_callback = tokenStatusCallback;
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    return;
  }
  char strftime_buf[64];
  strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
  Serial.print("Current time: ");
  Serial.println(strftime_buf);
}
String getLocation(double lat, double lon) {
    if (lat >= 29.8000 && lat <= 30.2200 && lon >= 31.1000 && lon <= 31.4500)
        return "Cairo";
    if (lat >= 29.9500 && lat <= 30.1000 && lon >= 31.5500 && lon <= 31.7500)
        return "New_Administrative_Capital";
    if (lat >= 31.1000 && lat <= 31.3500 && lon >= 29.7500 && lon <= 30.0500)
        return "Alexandria";
    if (lat >= 29.5000 && lat <= 29.7000 && lon >= 32.3000 && lon <= 32.4200)
        return "Ain_Sokhna";

    return "Unknown";
}

String Read(){
    String x = mySerial.readStringUntil('\n');
    x.trim();
    return x;
}
void loop() {
  if(mySerial.available()){
    String R = Read();
    if(R.length() > 8){
    char charArray[R.length() + 1];
    R.toCharArray(charArray, sizeof(charArray));
    char *token;
    char *rest = charArray;
    idx = 0;
    while ((token = strtok_r(rest, ",", &rest)) && idx < numValues) {
    values[idx] = atof(token);
    idx++;
    }
    idx=0;
    String add = "drums/"+getLocation(values[2] , values[3])+"/";
    Serial.println(add);
    while(idx < numValues && add != "drums/Unknown/"){
    if(idx == 0){
        time_t now;
        time(&now);
        String tt = String(now);
        tt = add+"wire_history/"+tt;
        Firebase.RTDB.setFloat(&fbdo, tt, values[idx]);
    }
    else if(idx == 1) Firebase.RTDB.setFloat(&fbdo, add+"temperature", values[idx]);
    else if(idx == 2) Firebase.RTDB.setFloat(&fbdo, add+"lat", values[idx]);
    else if(idx == 3){
      Firebase.RTDB.setFloat(&fbdo, add+"lng", values[idx]);
      Serial.println(values[idx]);
    }
    idx++;
  }
  }
  }
}

//cycles,temp,lat,lng
//----------------------------Fire Base----------------
#include "time.h"
#include <Firebase_ESP_Client.h>
#include <addons/TokenHelper.h>
#include <addons/RTDBHelper.h>
#define WIFI_SSID "ILFforever"
#define WIFI_PASSWORD "19283746"
#define API_KEY "AIzaSyDuteALFhATsyRtMggoGLMyhXedClzsZys"
#define DATABASE_URL "https://nsc-ic3-default-rtdb.asia-southeast1.firebasedatabase.app/"
#define STORAGE_BUCKET_ID "nsc-ic3.appspot.com"
#define USER_EMAIL "hammymukura@gmail.com"
#define USER_PASSWORD "onoffcomp2517"
bool Sendsuccess;
bool getlog = false; //for wifi logs
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
String uid;
unsigned long count = 0;
bool WifiRecon = true;
String CurWifi, CurPW;
//------------------------------
// Database child nodes
int timestamp;
FirebaseJson json;
const char *ntpServer = "pool.ntp.org";
void WifiCycle(const char Input);
int StartWifi;
bool WifiNotFound;
void WifiInit()
{
  Sendsuccess = false; //reset bool
  WiFi.mode(WIFI_STA);
  WiFi.begin(CurWifi.c_str(), CurPW.c_str());
  if (getlog)
  {
    Serial.println("CurWifi = " + CurWifi + " CurPW = " + CurPW);
    Serial.print("Free Heap : " + String(ESP.getFreeHeap()));
    Serial.print("Connecting to Wi-Fi");
  }
  StartWifi = millis();
  while (WiFi.status() != WL_CONNECTED)
  {
    if (millis() - StartWifi >= 10000) // take too long to find wifi
    {
      WifiNotFound = true;
      WiFi.mode(WIFI_MODE_NULL); // Turn off wifi
      break;
    }
    // delay(300);
  }
  if (!WifiNotFound)
  {
    if (getlog)
    {
      Serial.print("Connected with IP: ");
      Serial.println(WiFi.localIP());
    }
    configTime(0, 0, ntpServer);
    if (getlog)
      Serial.printf("Firebase Client v%s\n\n", FIREBASE_CLIENT_VERSION);
    config.api_key = API_KEY;
    auth.user.email = USER_EMAIL;
    auth.user.password = USER_PASSWORD;
    config.database_url = DATABASE_URL;

    config.token_status_callback = tokenStatusCallback; // see addons/TokenHelper.h
    config.max_token_generation_retry = 5;
    Firebase.begin(&config, &auth);
    while ((auth.token.uid) == "")
    {
      if (getlog)
        Serial.print('.');
      delay(500);
    }
    uid = auth.token.uid.c_str();
    if (getlog)
      Serial.print("User UID: " + String(uid));
    Firebase.reconnectWiFi(WifiRecon); // Comment or pass false value when WiFi reconnection will control by your code or third party library
  }
}

unsigned long getTime()
{
  time_t now;
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo))
  {
    // Serial.println("Failed to obtain time");
    return (0);
  }
  time(&now);
  return now;
}

void WifiCycle(String str, String Name) // Path and server side name;
{
  if (Firebase.ready())
  {
    timestamp = getTime();
    if (Firebase.Storage.upload(&fbdo, STORAGE_BUCKET_ID, str, mem_storage_type_sd, str, Name))
    {
       //Serial.printf("\nDownload URL: %s\n", fbdo.downloadURL().c_str());
       Sendsuccess = true;
    }
    else
    {
       //Serial.println(fbdo.errorReason());
       Sendsuccess = false;
    }
  }
}
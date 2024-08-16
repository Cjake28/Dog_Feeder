#include <Arduino.h>

// -----------------------------------------AP_SERVER--------------------------------------
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "esp_camera.h"
#include <ArduinoWebsockets.h>
#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"
#include <AsyncElegantOTA.h>
#include <AsyncElegantOTA.h>
#include <AsyncTCP.h>
// --------------------TASK_HANDLE-----------------------
TaskHandle_t wifiSetupHandle;
TaskHandle_t AutoconnectHandle;
TaskHandle_t connectToWebSocket_handle;
TaskHandle_t video_stream_handle;
TaskHandle_t FireStore_handle;
TaskHandle_t Manual_contorl_handle;
// --------------------FIREBASE--------------------------
#include <Firebase_ESP_Client.h>
#include <addons/TokenHelper.h>
#include <ArduinoJson.h>

#define FIREBASE_USE_PSRAM

#define API_KEY ""

#define FIREBASE_PROJECT_ID "dog-feeder"
AsyncWebServer server(80);

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

// --------------------TASK_HANDLE-----------------------
const char *apSSID = "Pet_Feeder";
const char *apPassword = "password123";

const char ssl_cert[]  = \
  "-----BEGIN CERTIFICATE-----\n" \
  "MIIDdzCCAl+gAwIBAgIEAgAAuTANBgkqhkiG9w0BAQUFADBaMQswCQYDVQQGEwJJ\n" \
  "RTESMBAGA1UEChMJQmFsdGltb3JlMRMwEQYDVQQLEwpDeWJlclRydXN0MSIwIAYD\n" \
  "VQQDExlCYWx0aW1vcmUgQ3liZXJUcnVzdCBSb290MB4XDTAwMDUxMjE4NDYwMFoX\n" \
  "DTI1MDUxMjIzNTkwMFowWjELMAkGA1UEBhMCSUUxEjAQBgNVBAoTCUJhbHRpbW9y\n" \
  "ZTETMBEGA1UECxMKQ3liZXJUcnVzdDEiMCAGA1UEAxMZQmFsdGltb3JlIEN5YmVy\n" \
  "VHJ1c3QgUm9vdDCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBAKMEuyKr\n" \
  "mD1X6CZymrV51Cni4eiVgLGw41uOKymaZN+hXe2wCQVt2yguzmKiYv60iNoS6zjr\n" \
  "IZ3AQSsBUnuId9Mcj8e6uYi1agnnc+gRQKfRzMpijS3ljwumUNKoUMMo6vWrJYeK\n" \
  "mpYcqWe4PwzV9/lSEy/CG9VwcPCPwBLKBsua4dnKM3p31vjsufFoREJIE9LAwqSu\n" \
  "XmD+tqYF/LTdB1kC1FkYmGP1pWPgkAx9XbIGevOF6uvUA65ehD5f/xXtabz5OTZy\n" \
  "dc93Uk3zyZAsuT3lySNTPx8kmCFcB5kpvcY67Oduhjprl3RjM71oGDHweI12v/ye\n" \
  "jl0qhqdNkNwnGjkCAwEAAaNFMEMwHQYDVR0OBBYEFOWdWTCCR1jMrPoIVDaGezq1\n" \
  "BE3wMBIGA1UdEwEB/wQIMAYBAf8CAQMwDgYDVR0PAQH/BAQDAgEGMA0GCSqGSIb3\n" \
  "DQEBBQUAA4IBAQCFDF2O5G9RaEIFoN27TyclhAO992T9Ldcw46QQF+vaKSm2eT92\n" \
  "9hkTI7gQCvlYpNRhcL0EYWoSihfVCr3FvDB81ukMJY2GQE/szKN+OMY3EU/t3Wgx\n" \
  "jkzSswF07r51XgdIGn9w/xZchMB5hbgF/X++ZRGjD8ACtPhSNzkE1akxehi/oCr0\n" \
  "Epn3o0WC4zxe9Z2etciefC7IpJ5OCBRLbf1wbWsaY71k5h+3zvDyny67G7fyUIhz\n" \
  "ksLi4xaNmjICq44Y3ekQEe5+NauQrz4wlHrQMz2nZQ/1/I6eYs9HRCwBXbsdtTLS\n" \
  "R9I4LtD+gdwyah617jzV/OeBHRnDJELqYzmp\n" \
  "-----END CERTIFICATE-----\n";

// TaskHandle_t wifiSetupTask; // Declare the task handle
#define Pbutton 12
#define motorRelayPin 4

unsigned long functionStartTime = 0;

bool functionRunning = false;
bool tog = false;
bool but = false;
//  -------------- LOGIC FUNCTION AND VAR FOR MANUAL CONTROL
using namespace websockets;
WebsocketsClient client;

#define SDA 15
#define SCL 14
#define incrementbutton 2
#define feedingbutton 13
// #define led 4


int count = 1;
bool firstcondition = true;
unsigned long buttonPressTime = 0;
bool endfeedingstate = false;

LiquidCrystal_I2C lcd(0x27, 16, 2); 


void lcdfirstcondition(){
  if(WiFi.status() != WL_CONNECTED){
    lcd.clear();
    lcd.setCursor(1, 0);
    lcd.print("NOT CONNECTED");
    lcd.setCursor(4, 1);
    lcd.print("TO WIFI");
    count = 1;
    return;
  }
  lcd.clear();
  lcd.setCursor(3, 0);
  lcd.print("CONNECTED");
  lcd.setCursor(4, 1);
  lcd.print("TO WIFI");
  // if(!firstcondition){
  //  firstcondition = true; 
  // }
}

void lcdsecondcondition(int counts){
  lcd.clear();
  lcd.setCursor(4,0);
  lcd.print("portion:");
  lcd.setCursor(7,1);
  lcd.print(counts);
  firstcondition = false;
}

void startfeeding(){
  lcd.clear();

  lcd.setCursor(4,0);
  lcd.print("feeding");
  digitalWrite(motorRelayPin, HIGH); 
}

void endfeeding(){
  lcd.clear();

  count = 1;
  endfeedingstate = false;
  firstcondition = true;
}

void Manual_control(void *pvParameters){
  
  for(;;){
    vTaskDelay(100 / portTICK_PERIOD_MS);
    int incrementstate = digitalRead(incrementbutton);
    int startfeedingbtn = digitalRead(feedingbutton);
    // Serial.println(incrementstate);

    if(firstcondition){
      lcdfirstcondition();
    //    Serial.print("Startfeeding: ");
    // Serial.println(startfeedingbtn);
    // Serial.print("incrementState: ");
    // Serial.println(incrementstate);
    }

    if(!incrementstate){
      Serial.println("Press buttonstate");
      if(count == 10){
        count = 1;
      }else{
        count++;
      }
      Serial.println(count);
      buttonPressTime = millis();
      lcdsecondcondition(count);
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    if(!startfeedingbtn){
      Serial.println("Start feedingbtn");
      startfeeding();
      vTaskDelay(450*count / portTICK_PERIOD_MS);
      endfeedingstate = true;
      digitalWrite(motorRelayPin, LOW);
    }else{
      if (millis() - buttonPressTime > 5000) {
          digitalWrite(motorRelayPin, LOW);
          count = 1;
          firstcondition = true;
      } 
    }

    if(endfeedingstate){
      endfeeding();
    }
    // Serial.println("asdss");
    // Serial.println(millis() - buttonPressTime);
  
  }
}

// logic function for firestore

void MotorOn(int Interval_open) {
  if (!functionRunning) {
    functionStartTime = millis();     
    functionRunning = true;           
    Serial.println("start");
    vTaskSuspend(Manual_contorl_handle);
    digitalWrite(motorRelayPin, HIGH);
  }

  if (millis() - functionStartTime >= Interval_open) {
    Serial.println("Function stopped after 4 seconds");
    Serial.println("stop");
    vTaskResume(Manual_contorl_handle);
    digitalWrite(motorRelayPin, LOW); 
    functionRunning = false;          
    tog = false;
    but = false;                      
  }
}


void FireStore(void *pvParameters) {
  bool motorOn= false;
  int portion = 1;
  String path = "device-feeder/DMX12242024";
  config.token_status_callback = tokenStatusCallback; 
  config.api_key = API_KEY;
  Firebase.signUp(&config, &auth,"","");
  Firebase.begin(&config, &auth);

  unsigned long prevmillis = 0;
    
  for (;;) {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi not connected.");
      vTaskDelay(2000 / portTICK_PERIOD_MS);
      continue;
    }

    if (!Firebase.ready()) {
      Serial.println("Firebase is not ready.");
      vTaskDelay(3000 / portTICK_PERIOD_MS);
      continue;
    }
  
    if (millis() - prevmillis >= 1500) {
      prevmillis = millis();
      Serial.println("--------------------------------");
      FirebaseJson query;

      if (Firebase.Firestore.getDocument(&fbdo, FIREBASE_PROJECT_ID, "", path.c_str(), "")) {
        // Serial.println(fbdo.payload().c_str());
        StaticJsonDocument
        <4096> doc;
        DeserializationError error = deserializeJson(doc, fbdo.payload().c_str(), DeserializationOption::NestingLimit(20));

        if (error) {
          Serial.print("deserializeJson() failed: ");
          Serial.println(error.c_str());
          // Handle deserialization error appropriately
          continue;
        }

        motorOn = doc["fields"]["motorOn"]["booleanValue"];
        portion = doc["fields"]["portion"]["integerValue"];
        
        Serial.println(motorOn);
        Serial.println(portion);
        // if (flashOn) {
        //   // digitalWrite(led, HIGH);
        // } else {
        // // digitalWrite(led, LOW);
        // }

        if (motorOn) {
          Serial.println("Turning motor off...");
          query.set("fields/motorOn/booleanValue", false);

          if (Firebase.Firestore.patchDocument(&fbdo, FIREBASE_PROJECT_ID, "", path.c_str(), query.raw(), "motorOn")) {
            Serial.println("Motor turned off successfully.");
            but = true;
          } else {
            Serial.println("Failed to turn off motor.");
            Serial.println(fbdo.errorReason());
            // Handle patch document failure appropriately
          }
        }
        Serial.println(Firebase.getFreeHeap());
      }
    }

    if (but) {
      tog = true;
    }

    if (tog) {
      // Serial.println(portion);

      // Serial.println(interval);
      // MotorOn(interval);
      vTaskSuspend(Manual_contorl_handle);
      digitalWrite(motorRelayPin, HIGH); 
      vTaskDelay(450*portion / portTICK_PERIOD_MS);
      vTaskResume(Manual_contorl_handle);
      digitalWrite(motorRelayPin, LOW); 
      functionRunning = false;          
      tog = false;
      but = false;    
    }
        // Add a delay before checking again
        vTaskDelay(150 / portTICK_PERIOD_MS);
  }
}


// long press button for turn on wificonfig
void longpressBttn(void *pvParameters){
  int counter = 0;
  while(1){

    bool bttnPush = digitalRead(Pbutton);
    // Serial.println(bttnPush);
    if(!bttnPush){
      
      counter++;

      Serial.println(counter);  
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      continue;
    }
    
      if(counter >= 5){
        Serial.println("Longpress");
        vTaskResume(wifiSetupHandle);
        vTaskSuspend(AutoconnectHandle);
        vTaskSuspend(connectToWebSocket_handle);
        vTaskSuspend(video_stream_handle);
        vTaskSuspend(FireStore_handle);
        vTaskSuspend(Manual_contorl_handle);
        counter = 0;
        vTaskDelay(40 / portTICK_PERIOD_MS);
      }else{
        counter = 0;
      }
    
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void saveCredentials(String ssid, String password) {
  String fileLine = "";
  File configFile = SPIFFS.open("/config.txt", "w");
  if (!configFile) {
    Serial.println("Failed to open config file for writing.");
    return;
  }

  configFile.print(ssid);
  configFile.print(" ");
  configFile.print(password);

  configFile.close();

  if (!SPIFFS.exists("/config.txt")) {
    Serial.println("File does not exist!");
    return;
  } 

  File file = SPIFFS.open("/config.txt", "r");

  if (!file) {
    Serial.println("File is initialize");
    return;
  }

  if(file.available()){
    fileLine = file.readString();
    // Serial.println("File Content: " + fileLine);
  }
  
  int separatorIndex = fileLine.indexOf(' ');
  String ssidSaved = fileLine.substring(0,separatorIndex);
  String passwordSaved = fileLine.substring(separatorIndex + 1);
  // Serial.println("aes "+ ssid + " " + ssidSaved);
  // Serial.println("eas " + password + " " + passwordSaved);

  if (ssid == ssidSaved && password == passwordSaved) {
    Serial.println("saved correct credentials");
  } else {
    Serial.println("saved incorrect credentials");
    return;
  }

  Serial.println("Credentials saved to config file.");
}


void stopServer() {
  // Stop the Access Point
  WiFi.softAPdisconnect(true);
  // AsyncElegantOTA.end();
  // Stop the server
  server.end();
  lcd.clear();
  vTaskSuspend(wifiSetupHandle);
  vTaskResume(AutoconnectHandle);
  vTaskResume(FireStore_handle);
  vTaskResume(Manual_contorl_handle);
  Serial.println("turned off AP and server");
}

bool stopWiFiSetupTask = false;

void setupWiFi(void *pvParameters) {
  stopWiFiSetupTask= false;
  // Create an Access Point
  vTaskDelay(200 / portTICK_PERIOD_MS);
  lcd.clear();
  lcd.setCursor(5,0);
  lcd.print("SETUP");
  lcd.setCursor(6,1);
  lcd.print("WIFI");

  vTaskDelay(1000 / portTICK_PERIOD_MS);

  WiFi.disconnect(true);
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(apSSID, apPassword);
  Serial.println("Access Point Started");
  AsyncElegantOTA.begin(&server);

  // Set up routes
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {

    lcd.clear();
    lcd.setCursor(6,0);
    lcd.print("PUT");
    lcd.setCursor(2,1);
    lcd.print("CREDENTIALS");

    String html = R"(
      <!DOCTYPE html>
      <html lang="en">
      <head>
        <meta charset="UTF-8">
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <title>Device WiFi Setup</title>
        <style>
          body {
            font-family: Arial, sans-serif;
            background-color: #f0f0f0;
            margin: 0;
            padding: 20px;
          }
          .container {
            max-width: 400px;
            margin: auto;
            background-color: #fff;
            padding: 20px;
            border-radius: 8px;
            box-shadow: 0 0 10px rgba(0, 0, 0, 0.1);
          }
          h1 {
            text-align: center;
          }
          label {
            display: block;
            margin-bottom: 5px;
            font-weight: bold;
          }
          input[type="text"],
          input[type="password"] {
            width: calc(100% - 12px);
            padding: 6px;
            margin-bottom: 10px;
            border: 1px solid #ccc;
            border-radius: 4px;
          }
          input[type="submit"] {
            width: 100%;
            padding: 8px;
            background-color: #4CAF50;
            color: white;
            border: none;
            border-radius: 4px;
            cursor: pointer;
            font-size: 16px;
          }
          input[type="submit"]:hover {
            background-color: #45a049;
          }
        </style>
      </head>
      <body>
        <div class="container">
          <h1>Device WiFi Setup</h1>
          <form action="/check" method="post">
            <label for="ssid">SSID:</label>
            <input type="text" id="ssid" name="ssid" required><br>
            <label for="password">Password:</label>
            <input type="password" id="password" name="password" required><br>
            <input type="submit" value="Check">
          </form>
        </div>
      </body>
      </html>
    )";

    request->send(200, "text/html", html);

});


server.on("/check", HTTP_POST, [](AsyncWebServerRequest *request) {
    String ssidToConnect = request->arg("ssid");
    String passwordToConnect = request->arg("password");
    lcd.clear();
    lcd.setCursor(5,0);
    lcd.print("CHECKING");

    // Redirect to /newpath with URL parameters
    String redirectURL = "/newpath?ssid=" + ssidToConnect + "&password=" + passwordToConnect;
    request->send(200, "text/html", "<!DOCTYPE html>"
                                     "<html lang=\"en\">"
                                     "<head>"
                                     "<meta charset=\"UTF-8\">"
                                     "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">"
                                     "<title>Checking</title>"
                                     "<style>"
                                     "body {"
                                     "  font-family: Arial, sans-serif;"
                                     "  background-color: #f0f0f0;"
                                     "  margin: 0;"
                                     "  padding: 20px;"
                                     "}"
                                     "h1 {"
                                     "  text-align: center;"
                                     "}"
                                     "p {"
                                     "  text-align: center;"
                                     "}"
                                     "</style>"
                                     "</head>"
                                     "<body>"
                                     "<h1>Checking...</h1>"
                                     "<p>Please wait while we check your Wi-Fi credentials...</p>"
                                     "<script>"
                                     "setTimeout(function(){ window.location.replace('" + redirectURL + "'); }, 5000);"
                                     "</script>"
                                     "</body>"
                                     "</html>");

    // Attempt to connect to Wi-Fi
    WiFi.begin(ssidToConnect.c_str(), passwordToConnect.c_str());
});


  
  server.on("/newpath", HTTP_GET, [](AsyncWebServerRequest *request) {
    // Retrieve parameters from the URL
    String ssid = request->arg("ssid");
    String password = request->arg("password");

    ssid.trim();
    password.trim();

    // Serial.println("SSIDtae: " + ssid);
    // Serial.println("Passwordeta: " + password);

    // Your logic here...
    if (WiFi.status() != WL_CONNECTED) {
      // Connection failed
      Serial.println("Failed to connect to Wi-Fi");

      request->redirect("/");
      return;
    }

    lcd.clear();
    lcd.setCursor(2,0);
    lcd.print("CONNECTED");
    saveCredentials(ssid, password);
    request->send(200, "text/html", "<h1>Connected to WIFI</h1>"
                                    "<script>setTimeout(function(){ window.location.replace('/close'); }, 2000);</script>");
  });

  server.on("/close", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->redirect("https://www.google.com");
    Serial.println("delete");
    lcd.clear();
    stopWiFiSetupTask = true;
  });
  bool tut = false;
  // Start server
  server.begin();
  while(1){
    AsyncElegantOTA.loop();
    if(stopWiFiSetupTask){
      // Your existing code
      Serial.println("turnof setupwifi freertos");
      // stopServer();
      // stopWiFiSetupTask = false;
      // vTaskDelay(100 / portTICK_PERIOD_MS);
      // vTaskSuspend(wifiSetupHandle);
      ESP.restart();
    }
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    Serial.println("notconnected: wifi setup");
    // vTaskDelay(pdMS_TO_TICKS(4000));
    // Serial.println("gago");
  }
}

void AutoConnect_wifi(void *pvParameters){
  vTaskDelay(500 / portTICK_PERIOD_MS);
  String fileline = "";
  WiFi.mode(WIFI_STA);
  vTaskDelay(500 / portTICK_PERIOD_MS);
  while(1){
    // Serial.println("AutoConnect_wifi Running");
    if(WiFi.status() == WL_CONNECTED){
      Serial.println("AutoConnect :connected");
      vTaskResume(connectToWebSocket_handle);
      vTaskResume(FireStore_handle);
      vTaskDelay(5000 / portTICK_PERIOD_MS); 
      continue;
    }
    // // Serial.println("disconnected");
    if (!SPIFFS.exists("/config.txt")) {
      Serial.println("File does not exist!");
      continue;
    }

    File file = SPIFFS.open("/config.txt", "r");

    if (!file) {
      Serial.println("File is initialize");
      // vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }

    if(file.available()){
      fileline = file.readString();
    // Serial.println("File Content: " + fileline);
    }

    int separatorIndex = fileline.indexOf(' ');
    String ssid = fileline.substring(0,separatorIndex);
    String password = fileline.substring(separatorIndex + 1);
    // Serial.println("aes " +  ssid);
    // Serial.println("eas " +  password);

    
    WiFi.begin(ssid,password);

    unsigned long StartAttemptTime = millis();

    while(WiFi.status() != WL_CONNECTED && millis() - StartAttemptTime < 10000){
      Serial.println("Connecting");
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    if(WiFi.status() != WL_CONNECTED){
      Serial.println("WIFI FAILED");
      continue;
    }

    // Serial.println("WIFI CONNECTED: " + WiFi.localIP());
  }
}
// -----------------------------------------AP_SERVER--------------------------------------


// -----------------------------------------WEBSOCKET--------------------------------------


void connectToWebSocket(void *pvParameters){
  vTaskDelay(500 / portTICK_PERIOD_MS);
  // client.setCACert(ssl_cert);
  const char* websocket_server_host = "192.168.137.1";
  const uint16_t websocket_server_port = 8898;
  while(1) {
    // Check the state of video_stream task
    if (video_stream_handle != NULL) {
      eTaskState taskState = eTaskGetState(video_stream_handle);
      if(client.available()){
        if(taskState == eSuspended){
          vTaskResume(video_stream_handle);   
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        continue;
      }else if (!client.available()){
        if(taskState == eRunning){
          vTaskSuspend(video_stream_handle);
        }
        // Attempt to connect to WebSocket server
        while (!client.connect(websocket_server_host, websocket_server_port, "/")){
          Serial.print(".");
          vTaskDelay(500 / portTICK_PERIOD_MS);
        }
        Serial.println("Websocket Connected!");
        // Resume video streaming task after successful connection
        vTaskResume(video_stream_handle);
      }
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }else{
    Serial.println("TAENA NULL");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    
   }
  }
}

void video_stream(void *pvParameters){
  vTaskDelay(500 / portTICK_PERIOD_MS);
  while(1){
    if(!client.available()){
      Serial.println("video stream Not connected websocket");
      vTaskDelay(3000 / portTICK_PERIOD_MS);
      continue;
    }

    camera_fb_t *fb = esp_camera_fb_get();
    if(!fb){
      Serial.println("Camera capture failed");
      esp_camera_fb_return(fb);
      continue;
    }

    if(fb->format != PIXFORMAT_JPEG){
      Serial.println("Non-JPEG data not implemented");
      continue;
    }
    // Serial.println("...");

    client.sendBinary((const char*) fb->buf, fb->len);
    esp_camera_fb_return(fb);
  }
}

// -----------------------------------------WEBSOCKET--------------------------------------


void autorun(void *pvParameters){

  for(;;){
    Serial.println("autoRun");
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA,SCL);
  Serial.setDebugOutput(true);
  lcd.init(); 
  lcd.backlight(); 
  pinMode(Pbutton, INPUT_PULLUP);
  pinMode(incrementbutton, INPUT_PULLUP);
  pinMode(feedingbutton, INPUT_PULLUP);
  // pinMode(led, OUTPUT);
  pinMode(motorRelayPin, OUTPUT);
  // motorRelay HIGH = ON
  digitalWrite(motorRelayPin, LOW);
  // digitalWrite(led, LOW);
  
  if (!SPIFFS.begin(true)) {
    Serial.println("An error occurred while mounting SPIFFS.");
    return;
  }

  // Create a task to set up Wi-Fi and handle connections
  if(psramInit()){
    Serial.println("\nPSRAM is correctly initialized");
  }else{
    Serial.println("PSRAM not available");
  }
  
  camera_config_t camconfig;
  camconfig.ledc_channel = LEDC_CHANNEL_0;
  camconfig.ledc_timer = LEDC_TIMER_0;
  camconfig.pin_d0 = Y2_GPIO_NUM;
  camconfig.pin_d1 = Y3_GPIO_NUM;
  camconfig.pin_d2 = Y4_GPIO_NUM;
  camconfig.pin_d3 = Y5_GPIO_NUM;
  camconfig.pin_d4 = Y6_GPIO_NUM;
  camconfig.pin_d5 = Y7_GPIO_NUM;
  camconfig.pin_d6 = Y8_GPIO_NUM;
  camconfig.pin_d7 = Y9_GPIO_NUM;
  camconfig.pin_xclk = XCLK_GPIO_NUM;
  camconfig.pin_pclk = PCLK_GPIO_NUM;
  camconfig.pin_vsync = VSYNC_GPIO_NUM;
  camconfig.pin_href = HREF_GPIO_NUM;
  camconfig.pin_sccb_sda = SIOD_GPIO_NUM;
  camconfig.pin_sscb_scl = SIOC_GPIO_NUM;
  camconfig.fb_location = CAMERA_FB_IN_PSRAM;
  camconfig.pin_pwdn = PWDN_GPIO_NUM;
  camconfig.pin_reset = RESET_GPIO_NUM;
  camconfig.xclk_freq_hz = 17000000;
  camconfig.pixel_format = PIXFORMAT_JPEG;
  //init with high specs to pre-allocate larger buffers
  if(psramFound()){
    // config.fb_location = CAMERA_FB_IN_PSRAM;
    camconfig.frame_size = FRAMESIZE_VGA;
    camconfig.jpeg_quality = 9;
    camconfig.fb_count = 4;
  } else {
    camconfig.frame_size = FRAMESIZE_SVGA;
    camconfig.jpeg_quality = 12;
    camconfig.fb_count = 1;
  }

  // camera init
  esp_err_t err = esp_camera_init(&camconfig);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
  
   xTaskCreatePinnedToCore(
      setupWiFi,                // Task function
      "SetupWiFi",              // Task name
      8192,                     // Stack size (adjust as needed)
      NULL,                     // Task parameters
      1,                        // Priority
      &wifiSetupHandle,           // Task handle
      0 // Core (use core 0)
  );

  xTaskCreatePinnedToCore(
    AutoConnect_wifi,                // Task function
    "AutoConnect_wifi",              // Task name
    8192,                     // Stack size (adjust as needed)
    NULL,                     // Task parameters
    2,                        // Priority
    &AutoconnectHandle,         // Task handle
    0    // Core (use core 0)
  );
  xTaskCreatePinnedToCore(
    longpressBttn,                // Task function
    "longpressBttn",              // Task name
    8192,                     // Stack size (adjust as needed)
    NULL,                     // Task parameters
    1,                        // Priority
    NULL,           // Task handle
    0 // Core (use core 0)
  );
  // Call the new function to connect to WebSocket
  xTaskCreatePinnedToCore(
    connectToWebSocket,                // Task function
    "connectToWebSocket",              // Task name
    8192,                     // Stack size (adjust as needed)
    NULL,                     // Task parameters
    2,                        // Priority
    &connectToWebSocket_handle,// Task handle
    0         
  );
  xTaskCreate(
    video_stream,                // Task function
    "video_stream",              // Task name
    8192,                     // Stack size (adjust as needed)
    NULL,                     // Task parameters
    5,                        // Priority
    &video_stream_handle
    // Task handle
  );

  xTaskCreate(
    FireStore,                // Task function
    "FireStore",              // Task name
    32768,                     // Stack size (adjust as needed)
    NULL,                     // Task parameters
    5,                        // Priority
    &FireStore_handle
    // Task handle
  );

  xTaskCreatePinnedToCore(
    Manual_control,                // Task function
    "Manual_control",              // Task name
    4096,                     // Stack size (adjust as needed)
    NULL,                     // Task parameters
    2,                        // Priority
    &Manual_contorl_handle,   // Task handle
    0
  );

  xTaskCreatePinnedToCore(
    autorun,                // Task function
    "autorun",              // Task name
    4096,                     // Stack size (adjust as needed)
    NULL,                     // Task parameters
    1,                        // Priority
    NULL,   // Task handle
    1
  );

  // vTaskStartScheduler();
  vTaskSuspend(wifiSetupHandle);
  vTaskResume(AutoconnectHandle);
  vTaskResume(Manual_contorl_handle);
  vTaskSuspend(FireStore_handle);
  vTaskSuspend(connectToWebSocket_handle);
  vTaskSuspend(video_stream_handle);
}

void loop(){
  delay(10000);
}


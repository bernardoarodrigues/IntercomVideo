/*
  Author: Bernardo Alexandre A. Rodrigues
  Compile as:
   ESP32 Dev Module
   CPU Freq: 240
   Flash Freq: 80
   Flash mode: QIO
   Flash Size: 4Mb
   Partition: Minimal SPIFFS
   PSRAM: Enabled
*/

// APPlication core e PROcess core (the one that runs ESP32 SDK stack)
#define APP_CPU 1
#define PRO_CPU 0

// *Libraries*

// Camera
#include "esp_camera.h"
#include "ov2640.h"
#include "camera_pins.h"

// Utilities
#include <WiFi.h>
#include <WebServer.h>
#include <WiFiClient.h>
#include <HTTPClient.h>
#include <EEPROM.h>
#include <ESPmDNS.h>
#include <ESP32SSDP.h>
#include <SPIFFS.h>
#include <esp_bt.h>
#include <esp_wifi.h>
#include <esp_sleep.h>
#include <driver/rtc_io.h>
#include <ArduinoOTA.h>

// SettingsServer
#include "SettingsServer.h"

// Firebase
#include <Firebase_ESP_Client.h>
#include <addons/TokenHelper.h>
#include <addons/RTDBHelper.h>

// *Variables*

// Server
WebServer server(80);
#define MAX_CLIENTS 10
String indexServer = "<!DOCTYPE html><html><head><meta charset=\"utf-8\"><title>Control Camera</title><style type=\"text/css\"> .cont { position: absolute; left: 50%; top: 50%; transform: translate(-50%, -50%); -ms-transform: translate(-50%, -50%); -webkit-transform: translate(-50%, -50%); text-align: center; } .lbl { color: #87857e; } </style></head><body><div class=\"cont\"><h1>Control Camera</h1><h3 class=\"lbl\">Device running...</h3></div></body></html>";

// GPIOs
#define flash 12
#define day 15
#define btn 2
uint8_t btn_prev;

// Firebase
String serial = "000000000001";
#define API_KEY "FIREBASE_API_KEY"
#define USER_PASSWORD "FIREBASE_USER_PASSWORD"
#define DATABASE_URL "FIREBASE_DATABASE_URL"
#define FIREBASE_FCM_SERVER_KEY "FIREBASE_FCM_SERVER_KEY"
#define STORAGE_BUCKET_ID "FIREBASE_STORAGE_BUCKET_ID"
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

// Root CA Certificate
const char rootCACert[] PROGMEM = "-----BEGIN CERTIFICATE-----\n"
                                  "MIIFVzCCAz+gAwIBAgINAgPlk28xsBNJiGuiFzANBgkqhkiG9w0BAQwFADBHMQsw\n"
                                  "CQYDVQQGEwJVUzEiMCAGA1UEChMZR29vZ2xlIFRydXN0IFNlcnZpY2VzIExMQzEU\n"
                                  "MBIGA1UEAxMLR1RTIFJvb3QgUjEwHhcNMTYwNjIyMDAwMDAwWhcNMzYwNjIyMDAw\n"
                                  "MDAwWjBHMQswCQYDVQQGEwJVUzEiMCAGA1UEChMZR29vZ2xlIFRydXN0IFNlcnZp\n"
                                  "Y2VzIExMQzEUMBIGA1UEAxMLR1RTIFJvb3QgUjEwggIiMA0GCSqGSIb3DQEBAQUA\n"
                                  "A4ICDwAwggIKAoICAQC2EQKLHuOhd5s73L+UPreVp0A8of2C+X0yBoJx9vaMf/vo\n"
                                  "27xqLpeXo4xL+Sv2sfnOhB2x+cWX3u+58qPpvBKJXqeqUqv4IyfLpLGcY9vXmX7w\n"
                                  "Cl7raKb0xlpHDU0QM+NOsROjyBhsS+z8CZDfnWQpJSMHobTSPS5g4M/SCYe7zUjw\n"
                                  "TcLCeoiKu7rPWRnWr4+wB7CeMfGCwcDfLqZtbBkOtdh+JhpFAz2weaSUKK0Pfybl\n"
                                  "qAj+lug8aJRT7oM6iCsVlgmy4HqMLnXWnOunVmSPlk9orj2XwoSPwLxAwAtcvfaH\n"
                                  "szVsrBhQf4TgTM2S0yDpM7xSma8ytSmzJSq0SPly4cpk9+aCEI3oncKKiPo4Zor8\n"
                                  "Y/kB+Xj9e1x3+naH+uzfsQ55lVe0vSbv1gHR6xYKu44LtcXFilWr06zqkUspzBmk\n"
                                  "MiVOKvFlRNACzqrOSbTqn3yDsEB750Orp2yjj32JgfpMpf/VjsPOS+C12LOORc92\n"
                                  "wO1AK/1TD7Cn1TsNsYqiA94xrcx36m97PtbfkSIS5r762DL8EGMUUXLeXdYWk70p\n"
                                  "aDPvOmbsB4om3xPXV2V4J95eSRQAogB/mqghtqmxlbCluQ0WEdrHbEg8QOB+DVrN\n"
                                  "VjzRlwW5y0vtOUucxD/SVRNuJLDWcfr0wbrM7Rv1/oFB2ACYPTrIrnqYNxgFlQID\n"
                                  "AQABo0IwQDAOBgNVHQ8BAf8EBAMCAYYwDwYDVR0TAQH/BAUwAwEB/zAdBgNVHQ4E\n"
                                  "FgQU5K8rJnEaK0gnhS9SZizv8IkTcT4wDQYJKoZIhvcNAQEMBQADggIBAJ+qQibb\n"
                                  "C5u+/x6Wki4+omVKapi6Ist9wTrYggoGxval3sBOh2Z5ofmmWJyq+bXmYOfg6LEe\n"
                                  "QkEzCzc9zolwFcq1JKjPa7XSQCGYzyI0zzvFIoTgxQ6KfF2I5DUkzps+GlQebtuy\n"
                                  "h6f88/qBVRRiClmpIgUxPoLW7ttXNLwzldMXG+gnoot7TiYaelpkttGsN/H9oPM4\n"
                                  "7HLwEXWdyzRSjeZ2axfG34arJ45JK3VmgRAhpuo+9K4l/3wV3s6MJT/KYnAK9y8J\n"
                                  "ZgfIPxz88NtFMN9iiMG1D53Dn0reWVlHxYciNuaCp+0KueIHoI17eko8cdLiA6Ef\n"
                                  "MgfdG+RCzgwARWGAtQsgWSl4vflVy2PFPEz0tv/bal8xa5meLMFrUKTX5hgUvYU/\n"
                                  "Z6tGn6D/Qqc6f1zLXbBwHSs09dR2CQzreExZBfMzQsNhFRAbd03OIozUhfJFfbdT\n"
                                  "6u9AWpQKXCBfTkBdYiJ23//OYb2MI3jSNwLgjt7RETeJ9r/tSQdirpLsQBqvFAnZ\n"
                                  "0E6yove+7u7Y/9waLd64NnHi/Hm3lCXRSHNboTXns5lndcEZOitHTtNCjv0xyBZm\n"
                                  "2tIMPNuzjsmhDYAPexZ3FL//2wmUspO8IFgV6dtxQ/PeEMMA3KgqlbbC1j+Qa3bb\n"
                                  "bP6MvPJwNQzcmRk13NfIRmPVNnGuV/u3gm3c\n"
                                  "-----END CERTIFICATE-----\n";

// Methods

// Checks Wifi connection
bool testWifi(void)
{
  int c = 0;
  while (c < 20)
  {
    if (WiFi.status() == WL_CONNECTED)
    {
      return true;
    }
    delay(500);
    c++;
  }
  return false;
}

// Gets msgTokens from database
const char *getTokens(void) {
  if (Firebase.ready()) {
    if (Firebase.RTDB.getString(&fbdo, "/devices/" + serial + "/msgtokens")) {
      const char *tokensStr = fbdo.to<const char *>();
      return tokensStr;
    } else {
      Serial.println(fbdo.errorReason().c_str());
    }
  }
}

void sendNotif() {
  // Gets tokens
  const char *dbTokens = getTokens();

  // If any token
  if (dbTokens != "") {
    Serial.println("got tokens");

    digitalWrite(flash, HIGH);
    
    // Takes photo and saves
    camera_fb_t *fb = esp_camera_fb_get();
    File file = SPIFFS.open("/photo.jpg", FILE_WRITE);
    file.write(fb->buf, fb->len);
    file.close();
    esp_camera_fb_return(fb);

    digitalWrite(flash, LOW);
    
    // Creates message
    FCM_Legacy_HTTP_Message msg;
    msg.options.priority = "high";

    // Verifies token count
    char *dbToken = strtok((char*)dbTokens, "/");
    int count = 0;
    while (dbToken != NULL) {
      count++;
      dbToken = strtok(NULL, "/");
    }

    char* tok = (char*)dbTokens;
    int i;

    // If only one
    if (count == 1) {
      Serial.println("one person");
      msg.targets.to = (char*)dbTokens;
    }
    // More than one
    else {
      FirebaseJsonArray arr;
      for (i = 0; i < count; ++i) {
        arr.add(tok);
        tok += strlen(tok) + 1;
        tok += strspn(tok, "/");
      }
      msg.targets.registration_ids = arr.raw();
      Serial.println("added folks");
    }

    // Creates data payload
    FirebaseJson payload;
    payload.add("entered", "false");

    // Uploads photo
    if (Firebase.Storage.upload(&fbdo, STORAGE_BUCKET_ID, "/photo.jpg", mem_storage_type_flash, "/photos/" + serial + ".jpg", "image/jpeg")) {
      String url = fbdo.downloadURL().c_str();
      payload.add("photo", url);
      Serial.println("add photo");
      msg.payloads.data = payload.raw();
    } else {
      payload.add("photo", "");
      msg.payloads.data = payload.raw();
    }

    // Sends message
    if (Firebase.FCM.send(&fbdo, &msg)) {
      Serial.println("Sent message");
    }
  }
}

// RTOS Tasks
TaskHandle_t tMjpeg;   // Handles client connections
TaskHandle_t tCam;     // Get camera frames and stores them

uint8_t       noActiveClients;
SemaphoreHandle_t frameSync = NULL;

// Desired fps
const int FPS = 24;

// Handles client request every 100ms
const int WSINTERVAL = 100;

// Task that handles clients
void mjpegCB(void* pvParameters) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(WSINTERVAL);

  // Creates frame sync semaphore and starts it
  frameSync = xSemaphoreCreateBinary();
  xSemaphoreGive( frameSync );

  // *Setup 2*

  // Creates RTOS Task to get camera frames
  xTaskCreatePinnedToCore(
    camCB,        // callback
    "cam",        // name
    4 * 1024,     // stack size
    NULL,         // params
    2,            // priority
    &tCam,        // RTOS task handler
    PRO_CPU);     // core

  // Server handler
  server.on("/", HTTP_GET, handle_index);
  server.on("/index", HTTP_GET, handle_index);
  server.on("/index.html", HTTP_GET, handle_index);
  server.on("/video", HTTP_GET, handleJPGSstream);
  server.on("/reset", HTTP_GET, handle_reset);
  server.on("/description.xml", HTTP_GET, handle_description);
  server.onNotFound(handleNotFound);

  // Begins MDNS
  if (!MDNS.begin("controlcamera")) {
    Serial.println("Error initializing MDNS");
    while (1) {
      delay(1000);
    }
  }
  
  // Adds MDNS service
  MDNS.addService("camera", "tcp", 8080);
  Serial.println("mDNS started");

  // Begins server
  server.begin();

  // Begins SSDP
  SSDP.setSchemaURL("description.xml");
  SSDP.setHTTPPort(80);
  SSDP.setName("Control Camera");
  SSDP.setSerialNumber("000000000001");
  SSDP.setURL("/index.html");
  SSDP.setModelName("ControlCameraV1");
  SSDP.setModelNumber("000000000001");
  SSDP.setModelURL("https://control-industries.web.app/");
  SSDP.setManufacturer("Control Industries");
  SSDP.setManufacturerURL("https://control-industries.web.app/");
  SSDP.setDeviceType("upnp:rootdevice");
  SSDP.begin();

  // Begins Spiffs
  if (!SPIFFS.begin(true)) {
    Serial.println("Error initializing SPIFFS");
  }

  // Begins ArduinoOTA
  startOTA();

  // Firebase Configuration
  config.api_key = API_KEY;
  auth.user.email = serial + "@gmail.com";
  auth.user.password = USER_PASSWORD;
  config.database_url = DATABASE_URL;
  config.cert.data = rootCACert;
  config.token_status_callback = tokenStatusCallback;
  config.max_token_generation_retry = 5;
  config.signer.preRefreshSeconds = 5 * 60;
  fbdo.setCert(rootCACert);
  fbdo.setResponseSize(100000);

  // Begins Firebase
  Firebase.begin(&config, &auth);
  Firebase.FCM.setServerKey(FIREBASE_FCM_SERVER_KEY);

  noActiveClients = 0;

  // Confirms setup 2
  for (int i = 0; i < 2; ++i) {
    digitalWrite(flash, HIGH);
    delay(200);
    digitalWrite(flash, LOW);
    delay(100);
  }

  // *Loop 2*
  xLastWakeTime = xTaskGetTickCount();
  for (;;) {
    // Handles ArduinoOTA
    ArduinoOTA.handle();

    // If button was pressed
    uint8_t btnVal = digitalRead(btn);
    if (btnVal == LOW && btn_prev == HIGH) {
      sendNotif();
    }
    btn_prev = digitalRead(btn);

    // Handles Client
    server.handleClient();

    // After handling client requests, let other tasks run
    taskYIELD();
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// Current frame information
volatile uint32_t frameNumber;
volatile size_t   camSize;      // Frame size, em bytes
volatile char*    camBuf;       // Pointer to current frame


// RTOS Task to get camera frames
void camCB(void* pvParameters) {
  TickType_t xLastWakeTime;
  
  // A running interval associated with currently desired frame rate
  const TickType_t xFrequency = pdMS_TO_TICKS(1000 / FPS);

  // Pointers to the 2 frames, their respective sizes and index of the current frame
  char* fbs[2] = { NULL, NULL };
  size_t fSize[2] = { 0, 0 };
  int ifb = 0;
  frameNumber = 0;

  // Camera Loop 
  xLastWakeTime = xTaskGetTickCount();

  for (;;) {
    // If it's dark turns flash on, else turns it off
    digitalRead(day) == HIGH ? digitalWrite(flash, HIGH) : digitalWrite(flash, LOW);
      
    // Grab a frame from the camera and query its size
    camera_fb_t* fb = NULL;
    fb = esp_camera_fb_get();
    size_t s = fb->len;

    // If frame size is more that we have previously allocated - increases allocated size
    if (s > fSize[ifb]) {
      fSize[ifb] = s + s;
      fbs[ifb] = allocateMemory(fbs[ifb], fSize[ifb]);
    }

    // Copy current frame into local buffer
    char* b = (char *)fb->buf;
    memcpy(fbs[ifb], b, s);
    esp_camera_fb_return(fb);

    // Let other tasks run and wait until the end of the current frame rate interval (if any time left)
    taskYIELD();
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    // Do not allow interrupts while switching the current frame
    xSemaphoreTake( frameSync, xFrequency );
    camBuf = fbs[ifb];
    camSize = s;
    ifb++;
    ifb &= 1;
    frameNumber++;
    
    // Let anyone waiting for a frame know that the frame is ready
    xSemaphoreGive( frameSync );

    // Lets other tasks run
    taskYIELD();

    //  If streaming task has suspended itself (no active clients to stream to)
    //  there is no need to grab frames from the camera, suspends the task
    if ( noActiveClients == 0 ) {
      // Passing NULL means "suspend yourself"
      vTaskSuspend(NULL);
    }
  }
}

// Memory allocator that takes advantage of PSRAM if present
char* allocateMemory(char* aPtr, size_t aSize) {

  // Since current buffer is too smal, free it
  if (aPtr != NULL) free(aPtr);

  char* ptr = NULL;
  ptr = (char*) ps_malloc(aSize);

  // Finally, if the memory pointer is NULL, we were not able to allocate any memory, and that is a terminal condition.
  if (ptr == NULL) {
    delay(5000);
    ESP.restart();
  }
  return ptr;
}


// Streaming Header
const char HEADER[] = "HTTP/1.1 200 OK\r\n" \
                      "Access-Control-Allow-Origin: *\r\n" \
                      "Content-Type: multipart/x-mixed-replace; boundary=123456789000000000000987654321\r\n";
const char BOUNDARY[] = "\r\n--123456789000000000000987654321\r\n";
const char CTNTTYPE[] = "Content-Type: image/jpeg\r\nContent-Length: ";
const int hdrLen = strlen(HEADER);
const int bdrLen = strlen(BOUNDARY);
const int cntLen = strlen(CTNTTYPE);

// StreamInfo Struct
struct streamInfo {
  uint32_t        frame;
  WiFiClient      client;
  TaskHandle_t    task;
  char*           buffer;
  size_t          len;
};

// Index
void handle_index(void)
{
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "text/html", indexServer);
}

// Video
void handleJPGSstream(void)
{
  // Can only acommodate 10 clients
  if ( noActiveClients >= MAX_CLIENTS ) return;

  // Stream Info
  streamInfo* info = new streamInfo;
  info->frame = frameNumber - 1;
  info->client = server.client();
  info->buffer = NULL;
  info->len = 0;

  // Creates task to stream to clients
  int rc = xTaskCreatePinnedToCore(
             streamCB,
             "strmCB",
             3 * 1024,
             (void*) info,
             2,
             &info->task,
             APP_CPU);
  if ( rc != pdPASS ) {
    delete info;
  }

  // Increases clients count
  noActiveClients++;

  // Wake up streaming tasks, if they were previously suspended
  if ( eTaskGetState( tCam ) == eSuspended ) vTaskResume( tCam );
}

// Streams content to all connected client
void streamCB(void * pvParameters) {
  char buf[16];
  TickType_t xLastWakeTime;
  TickType_t xFrequency;

  streamInfo* info = (streamInfo*) pvParameters;

  // Sends header to client
  info->client.write(HEADER, hdrLen);
  info->client.write(BOUNDARY, bdrLen);
  taskYIELD();

  xLastWakeTime = xTaskGetTickCount();
  xFrequency = pdMS_TO_TICKS(1000 / FPS);

  for (;;) {
    // Check if this client is still connected
    if ( info->client.connected() ) {
      if ( info->frame != frameNumber) {
        xSemaphoreTake( frameSync, portMAX_DELAY );
        if ( info->buffer == NULL ) {
          info->buffer = allocateMemory (info->buffer, camSize);
          info->len = camSize;
        }
        else {
          if ( camSize > info->len ) {
            info->buffer = allocateMemory (info->buffer, camSize);
            info->len = camSize;
          }
        }
        memcpy(info->buffer, (const void*) camBuf, info->len);
        xSemaphoreGive( frameSync );
        taskYIELD();

        info->frame = frameNumber;
        info->client.write(CTNTTYPE, cntLen);
        sprintf(buf, "%d\r\n\r\n", info->len);
        info->client.write(buf, strlen(buf));
        info->client.write((char*) info->buffer, (size_t)info->len);
        info->client.write(BOUNDARY, bdrLen);
        info->client.flush();
      }
    }
    else {
      // Client disconnected
      noActiveClients--;
      info->client.flush();
      info->client.stop();
      if ( info->buffer ) {
        free( info->buffer );
        info->buffer = NULL;
      }
      delete info;
      info = NULL;
      vTaskDelete(NULL);
    }
    
    // Let other tasks run after serving every client
    taskYIELD();
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// Reset
void handle_reset(void)
{
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "text/plain", "Ok");
  
  EEPROM.begin(512);
  for (int i = 0; i < 512; i++) {
    EEPROM.write(i, 0);
  }
  EEPROM.commit();
  
  ESP.restart();
}

// SSDP Description
void handle_description()
{
  SSDP.schema(server.client());
}

// Not Found
void handleNotFound()
{
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "text/plain", "Page not found");
}

void startOTA() {
  ArduinoOTA
  .onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else
      type = "filesystem";
    Serial.println("Start updating " + type);
  })
  .onEnd([]() {
    Serial.println("nEnd");
  })
  .onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%r", (progress / (total / 100)));
  })
  .onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
}

// *Setup 1*
void setup()
{
  // Begins Serial
  Serial.begin(115200);

  // Begins GPIOs
  pinMode(flash, OUTPUT);
  pinMode(day, INPUT_PULLUP);
  pinMode(btn, INPUT_PULLUP);

  // Configures GPIOs
  digitalWrite(flash, LOW);
  digitalWrite(btn, HIGH);
  btn_prev = digitalRead(btn);

  // Configures camera
  static camera_config_t camera_config = {
    .pin_pwdn       = PWDN_GPIO_NUM,
    .pin_reset      = RESET_GPIO_NUM,
    .pin_xclk       = XCLK_GPIO_NUM,
    .pin_sscb_sda   = SIOD_GPIO_NUM,
    .pin_sscb_scl   = SIOC_GPIO_NUM,
    .pin_d7         = Y9_GPIO_NUM,
    .pin_d6         = Y8_GPIO_NUM,
    .pin_d5         = Y7_GPIO_NUM,
    .pin_d4         = Y6_GPIO_NUM,
    .pin_d3         = Y5_GPIO_NUM,
    .pin_d2         = Y4_GPIO_NUM,
    .pin_d1         = Y3_GPIO_NUM,
    .pin_d0         = Y2_GPIO_NUM,
    .pin_vsync      = VSYNC_GPIO_NUM,
    .pin_href       = HREF_GPIO_NUM,
    .pin_pclk       = PCLK_GPIO_NUM,
    .xclk_freq_hz   = 20000000,
    .ledc_timer     = LEDC_TIMER_0,
    .ledc_channel   = LEDC_CHANNEL_0,
    .pixel_format   = PIXFORMAT_JPEG,
    .frame_size     = FRAMESIZE_CIF,
    .jpeg_quality   = 12,
    .fb_count       = 2
  };

  // Starts camera
  if (esp_camera_init(&camera_config) != ESP_OK) {
    delay(10000);
    ESP.restart();
  }
  sensor_t* s = esp_camera_sensor_get();
  s->set_vflip(s, false);

  // Confirms Setup 1
  digitalWrite(flash, HIGH);
  delay(400);
  digitalWrite(flash, LOW);
  delay(200);

  // Disconnects Wifi and starts EEPROM
  WiFi.disconnect();
  EEPROM.begin(512);
  delay(10);

  // Checks if any data in EEPROM
  boolean hasData = false;
  for (int i = 0; i < 150; ++i)
  {
    if(char(EEPROM.read(i)) != 0) {
      hasData = true;
      break;
    }
  }

  // If any credential saved in the memory
  if(hasData) {
    // Reads EEPROM

    // SSID
    String esid;
    for (int i = 0; i < 32; ++i)
    {
      esid += char(EEPROM.read(i));
    }
    // Password
    String epass;
    for (int i = 32; i < 64; ++i)
    {
      epass += char(EEPROM.read(i));
    }
    
    boolean ok = false;

    // While not connected
    while(!ok) {
      // Tries to connect to Wifi
      WiFi.begin(esid.c_str(), epass.c_str());
    
      // If connection was successful
      if (testWifi())
      {
        // Resets config state
        EEPROM.write(200, 0);
        EEPROM.commit();
      
        // Begins main RTOS Task
        xTaskCreatePinnedToCore(
          mjpegCB,
          "mjpeg",
          7 * 1024,
          NULL,
          2,
          &tMjpeg,
          APP_CPU);

        ok = true;
      }
      // If couldn't connect, waits 3s to try again
      else {
        // If in config, and coutldn't connect - wrong password
        if(char(EEPROM.read(200)) == 1)
          break;
        // Else - wifi network not working
        else 
          delay(3000);
      }
    }

    // Resets config state, and restarts configuring wifi
    if(char(EEPROM.read(200)) == 1) {
      EEPROM.write(200, 0);
      EEPROM.commit();
    }
    // Finishes setup and proceeds
    else {
      return;
    }
  }
  
  // If no credentials saved, starts server
  SettingsServer server;
  server.start();
}

// Loop 1
void loop() {
  if (WiFi.status() == WL_CONNECTED)
  {
    vTaskDelay(1000);
  }
}
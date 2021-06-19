/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/telegram-esp32-cam-photo-arduino/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/

#include <Arduino.h>
#include <WiFi.h>
#include <AutoConnect.h>
#include <WebServer.h>        
#include <WiFiClientSecure.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_camera.h"
#include <UniversalTelegramBot.h>
#include <ArduinoJson.h>

#define cameraIcon  "\xF0\x9F\x93\xB7"
#define flashIcon  "\xF0\x9F\x92\xA1"
#define onIcon  "\xE2\x9C\x85"
#define offIcon  "\xE2\x9D\x8C"
#define disarmIcon "\xF0\x9F\x92\xA4"
#define armIcon "\xF0\x9F\x93\xA2"
#define questionIcon "\xE2\x9D\x93"
#define lightIcon "\xF0\x9F\x94\xA6"
#define warnIcon "\xE2\x9D\x97"
#define wifiIcon "\xF0\x9F\x93\xB6"
#define rebootIcon "\xF0\x9F\x94\x83"

String message;

WebServer Server;
AutoConnect Portal(Server);
const char* ssid = "ESP_Cam";  // ssid (exemple)
const char* password = "Mot2passe";  // Mot de passe (exemple)
String  BOTtoken="1612760035:AAFQ_GhdHWJp-COviZbX2haCpHic3hFRchc";  // your Bot Token (Get from Botfather)

// Use @myidbot to find out the chat ID of an individual or a group
String  CHAT_ID="1767348642";

bool sendPhoto = false;
int armed=1;
int light=0;
bool restart = 0;

WiFiClientSecure clientTCP;
int wifi_counter = 0;
int wifi_try = 20;
UniversalTelegramBot bot(BOTtoken, clientTCP);

#define FLASH_LED_PIN 4
const int motionSensor = 13;
bool flashState = LOW;

//Checks for new messages every 1 second.
int botRequestDelay = 1000;
unsigned long lastTimeBotRan;

//CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22


void configInitCamera(){
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  //init with high specs to pre-allocate larger buffers
  if(psramFound()){
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;  //0-63 lower number means higher quality
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;  //0-63 lower number means higher quality
    config.fb_count = 1;
  }
  
  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    delay(1000);
    ESP.restart();
  }

  // Drop down frame size for higher initial frame rate
  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_CIF);  // UXGA|SXGA|XGA|SVGA|VGA|CIF|QVGA|HQVGA|QQVGA
}

void restartESP() {
  esp_sleep_enable_timer_wakeup(5000000);
  esp_deep_sleep_start();
}

void handleNewMessages(int numNewMessages) {
  Serial.print("Handle New Messages: ");
  Serial.println(numNewMessages);

  for (int i = 0; i < numNewMessages; i++) {
    String chat_id = String(bot.messages[i].chat_id);
    if (chat_id != CHAT_ID){
      bot.sendMessage(chat_id, "Unauthorized user", "");
      continue;
    }
    
    // Print the received message
    String text = bot.messages[i].text;
    Serial.println(text);
    
    String from_name = bot.messages[i].from_name;
    if (text == "/start") {
      String welcome = "Welcome, " + from_name + ".\n";
      welcome += "Use the following commands to control your outputs.\n\n";
      welcome += cameraIcon;
      welcome += " /photo to take a picture\n";
      welcome += armIcon;
      welcome += " /arm to arm the security system\n";
      welcome += disarmIcon;
      welcome += " /disarm to disarm the system\n";
      welcome += lightIcon;
      welcome += " /motionlight to trigger flash on motion\n";
      welcome += flashIcon;
      welcome += " /flash to toggle flash\n";
      welcome += questionIcon;
      welcome += " /state to request current flash and security system state \n";
      welcome += rebootIcon;
      welcome += " /reboot to reboot ESP \n";
      welcome += wifiIcon;
      welcome += " /config_reset to reset all wifi configuration \n";
      bot.sendMessage(CHAT_ID, welcome, "");
    }
    if (text == "/flash") {
      flashState = !flashState;
      digitalWrite(FLASH_LED_PIN, flashState);
      Serial.println("Change flash LED state");
      if(flashState){
        message = onIcon;
        message += " Flash is ";
        message += "ON";
      }
      else{
        message = offIcon;
        message += " Flash is ";
        message += "OFF";
      }
      bot.sendMessage(chat_id, message, "");
    }
    if (text == "/photo") {
      sendPhoto = true;
      Serial.println("New photo request");
    }
    if (text == "/arm") {
      armed = 1;
      message = armIcon;
      message += " Security System is ON";
      bot.sendMessage(chat_id, message, "");
    }
    if (text == "/disarm") {
      armed = 0;
      message = disarmIcon;
      message += " Security System is OFF";
      bot.sendMessage(chat_id, message, "");
    }
    /*if (text == "/reboot") {
      message = rebootIcon;
      message += " Rebooting ESP...";
      bot.sendMessage(chat_id, message, "");
      ESP.restart();
    }*/
    if (text == "/wifi") {
      message = wifiIcon;
      message += " Erasing wifi configuration...";
      bot.sendMessage(chat_id, message, "");
      WiFi.disconnect(true,true);
      delay(1000);   
      restart = 1;
    } 
    if (text == "/motionlight") {
      light = !light;
      if(light){
        message = onIcon;
        message += " Light on motion is ";
        message += "ON";
      }
      else{
        message = offIcon;
        message += " Light on motion is ";
        message += "OFF";
      }
      bot.sendMessage(chat_id, message, "");
    }
    if (text == "/state") {
      if (flashState){
        message = onIcon;
        message += " LED is ON";
        bot.sendMessage(chat_id, message, "");
      }
      else{
        message = offIcon;
        message += " LED is OFF ";
        bot.sendMessage(chat_id, message, "");
      }
      if(armed == 1){
        message = onIcon;
        message += " The security system is ON";
        bot.sendMessage(chat_id, message, "");
      }
      else {
        message = offIcon;
        message += " The security system is OFF";
        bot.sendMessage(chat_id, message, "");  
      }
      if (light){
        message = onIcon;
        message += " Light on motion is ON";
        bot.sendMessage(chat_id, message, "");
      }
      else{
        message = offIcon;
        message += " Light on motion is OFF ";
        bot.sendMessage(chat_id, message, "");
      }
    }
  }
}

void warnUser(){
  message = warnIcon;
  message += " Motion detected ";
  message += warnIcon;
  bot.sendMessage(CHAT_ID, message, "");
}

String sendPhotoTelegram() {
  const char* myDomain = "api.telegram.org";
  String getAll = "";
  String getBody = "";

  camera_fb_t * fb = NULL;
  fb = esp_camera_fb_get();  
  if(!fb) {
    Serial.println("Camera capture failed");
    delay(1000);
    ESP.restart();
    return "Camera capture failed";
  }  
  
  Serial.println("Connect to " + String(myDomain));


  if (clientTCP.connect(myDomain, 443)) {
    Serial.println("Connection successful");
    
    String head = "--RandomNerdTutorials\r\nContent-Disposition: form-data; name=\"chat_id\"; \r\n\r\n" + CHAT_ID + "\r\n--RandomNerdTutorials\r\nContent-Disposition: form-data; name=\"photo\"; filename=\"esp32-cam.jpg\"\r\nContent-Type: image/jpeg\r\n\r\n";
    String tail = "\r\n--RandomNerdTutorials--\r\n";

    uint16_t imageLen = fb->len;
    uint16_t extraLen = head.length() + tail.length();
    uint16_t totalLen = imageLen + extraLen;
  
    clientTCP.println("POST /bot"+BOTtoken+"/sendPhoto HTTP/1.1");
    clientTCP.println("Host: " + String(myDomain));
    clientTCP.println("Content-Length: " + String(totalLen));
    clientTCP.println("Content-Type: multipart/form-data; boundary=RandomNerdTutorials");
    clientTCP.println();
    clientTCP.print(head);
  
    uint8_t *fbBuf = fb->buf;
    size_t fbLen = fb->len;
    for (size_t n=0;n<fbLen;n=n+1024) {
      if (n+1024<fbLen) {
        clientTCP.write(fbBuf, 1024);
        fbBuf += 1024;
      }
      else if (fbLen%1024>0) {
        size_t remainder = fbLen%1024;
        clientTCP.write(fbBuf, remainder);
      }
    }  
    
    clientTCP.print(tail);
    
    esp_camera_fb_return(fb);
    
    int waitTime = 10000;   // timeout 10 seconds
    long startTimer = millis();
    boolean state = false;
    
    while ((startTimer + waitTime) > millis()){
      Serial.print(".");
      delay(100);      
      while (clientTCP.available()) {
        char c = clientTCP.read();
        if (state==true) getBody += String(c);        
        if (c == '\n') {
          if (getAll.length()==0) state=true; 
          getAll = "";
        } 
        else if (c != '\r')
          getAll += String(c);
        startTimer = millis();
      }
      if (getBody.length()>0) break;
    }
    clientTCP.stop();
    Serial.println(getBody);
  }
  else {
    getBody="Connected to api.telegram.org failed.";
    Serial.println("Connected to api.telegram.org failed.");
  }
  return getBody;
}

void setup(){
  armed = 1;
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); 
  // Init Serial Monitor
  Serial.begin(115200);

  // Set LED Flash as output
  pinMode(FLASH_LED_PIN, OUTPUT);
  digitalWrite(FLASH_LED_PIN, flashState);
  pinMode(GPIO_NUM_13, INPUT_PULLUP);

  // Config and init the camera
  configInitCamera();


  // Connect to Wi-Fi
  Serial.println();
  Serial.print("Connecting");
  WiFi.mode(WIFI_STA);
  clientTCP.setCACert(TELEGRAM_CERTIFICATE_ROOT); // Add root certificate for api.telegram.org
  if(Portal.begin()){
    Serial.println();
    Serial.print("ESP32-CAM IP Address: ");
    Serial.println(WiFi.localIP());  
  }
}

void loop() {
  Portal.handleRequest();
  if(WiFi.status() != WL_CONNECTED || restart==1) {
    restartESP();
  }
  if(armed == 1 && light == 1){
    int isDetected = digitalRead(motionSensor);
    if(isDetected == 1){
      digitalWrite(FLASH_LED_PIN,HIGH);
      delay(1000);
      sendPhotoTelegram();
      //warnUser();
      delay(1000);
      digitalWrite(FLASH_LED_PIN,LOW);
      delay(2000);
    }
  }

  if(armed == 1 && light == 0){
    int isDetected = digitalRead(motionSensor);
    if(isDetected == 1){
      sendPhotoTelegram();
      //warnUser();
      delay(2000);
    }
  }
  
  if (sendPhoto) {
    Serial.println("Preparing photo");
    sendPhotoTelegram(); 
    sendPhoto = false; 
  }
  if (millis() > lastTimeBotRan + botRequestDelay)  {
    int numNewMessages = bot.getUpdates(bot.last_message_received + 1);
    while (numNewMessages) {
      Serial.println("got response");
      handleNewMessages(numNewMessages);
      numNewMessages = bot.getUpdates(bot.last_message_received + 1);
    }
    lastTimeBotRan = millis();
  }
}

#include <Wire.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <WebSocketsClient.h>

#include "SparkFun_BNO080_Arduino_Library.h"  // Click here to get the library: https://github.com/sparkfun/SparkFun_BNO080_Arduino_Library
#define I2C_BUFFER_LENGTH 128


BNO080 myIMU;

String mac_address;

WiFiMulti WiFiMulti;
WebSocketsClient webSocket;

int fps = 60;
int port = 80;


// Choose only one!
//String bone = "LeftArm";
 String bone = "LeftForeArm";
// String bone = "LeftHand";
// String bone = "LeftUpLeg";
// String bone = "LeftLeg";
// String bone = "RightArm";
// String bone = "RightForeArm";
// String bone = "RightHand";
// String bone = "RightUpLeg";
// String bone = "RightLeg";
// String bone = "Spine";
// String bone = "Neck";
// String bone = "Head";
// String bone = "Hips";



// ID wifi to connect to
const char *ssid = "NETGEAR31";
const char *password = "fluffywind2904";
String serverIP = "192.168.0.130";
int sensor_clock = 9;  // updated clock - double check your soldering
int sensor_data = 8;   // this is from the soldering. double check what you have soldered your data to

float batt_v;
float quatI, quatJ, quatK, quatReal;


boolean start = false;


struct Quat {
  float x;
  float y;
  float z;
  float w;
} quat;
struct Euler {
  float x;
  float y;
  float z;
} euler;
char buff[256];
bool rtcIrq = false;
bool initial = 1;
bool otaStart = false;

uint8_t func_select = 0;
uint8_t omm = 99;
uint8_t xcolon = 0;
uint32_t targetTime = 0;  // for next 1 second timeout
uint32_t colour = 0;
int vref = 1100;

bool pressed = false;
uint32_t pressedTime = 0;
bool charge_indication = false;

uint8_t hh, mm, ss;
int pacnum = 0;


void hexdump(const void *mem, uint32_t len, uint8_t cols = 16) {
  const uint8_t *src = (const uint8_t *)mem;
  Serial.printf("\n[HEXDUMP] Address: 0x%08X len: 0x%X (%d)", (ptrdiff_t)src, len, len);
  for (uint32_t i = 0; i < len; i++) {
    if (i % cols == 0) {
      Serial.printf("\n[0x%08X] 0x%08X: ", (ptrdiff_t)src, i);
    }
    Serial.printf("%02X ", *src);
    src++;
  }
  Serial.printf("\n");
}

/*
 * Event handling for the websocket.  
 * This is from WebSocketClient.h 
 */

void webSocketEvent(WStype_t type, uint8_t *payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:  //when disconnected
      Serial.printf("[WSc] Disconnected!\n");
      break;
    case WStype_CONNECTED:  //when connected
      Serial.printf("[WSc] Connected to url: %s\n", payload);
      // send message to server when Connected
      webSocket.sendTXT("Connected");  //validation that you've connected
      webSocket.enableHeartbeat(1000, 100, 100);
      break;
    case WStype_TEXT:  //when you get a message
      Serial.printf("[WSc] get text: %s\n", payload);
      // send message to server
      // webSocket.sendTXT("message here");
      break;
    case WStype_BIN:
      Serial.printf("[WSc] get binary length: %u\n", length);
      hexdump(payload, length);
      // send data to server
      // webSocket.sendBIN(payload, length);
      break;
    case WStype_ERROR:
    case WStype_FRAGMENT_TEXT_START:
    case WStype_FRAGMENT_BIN_START:
    case WStype_FRAGMENT:
    case WStype_FRAGMENT_FIN:
      break;
  }
}


// define two tasks for Blink & AnalogRead
void TaskWifi(void *pvParameters);
void TaskReadIMU(void *pvParameters);

#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif





void setup() {

  Serial.begin(115200);
  delay(500);


  WiFiMulti.addAP(ssid, password);

  Serial.println("Connecting");

  while (WiFiMulti.run() != WL_CONNECTED) {
    delay(100);
    Serial.print(".");
  }

  Serial.println("");
  Serial.print("Connected to WiFi network with IP Address: ");  //this will be the local IP
  Serial.println(WiFi.localIP());

  mac_address = WiFi.macAddress();
  Serial.println(mac_address);


  delay(100);

  Wire.flush();
  delay(100);
  Wire.begin(sensor_clock, sensor_data);
  // Wire.setClock(1000000);

  myIMU.begin(BNO080_DEFAULT_ADDRESS, Wire);

  if (myIMU.begin() == false) {
    Serial.println("BNO080 not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
    while (1)
      ;
  }

  Wire.setClock(400000);

  myIMU.enableRotationVector(50);
  myIMU.enableAccelerometer(50);
  myIMU.enableGyro(50);
  myIMU.enableMagnetometer(50);

  Serial.println(F("IMU enabled"));



  delay(500);

  webSocket.begin(serverIP, port, "/hub");

  // event handler
  webSocket.onEvent(webSocketEvent);

  // use HTTP Basic Authorization this is optional remove if not needed
  // webSocket.setAuthorization("user", "Password");

  // try ever 5000 again if connection has failed
  webSocket.setReconnectInterval(0);
  webSocket.sendTXT(String(millis()).c_str());


  xTaskCreatePinnedToCore(
    TaskWifi, "TaskWifi"  // A name just for humans
    ,
    10000  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,
    NULL, 1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,
    NULL, 0);

  xTaskCreatePinnedToCore(
    TaskReadIMU, "TaskReadIMU", 10000  // Stack size
    ,
    NULL, 1  // Priority
    ,
    NULL, 1);
}

void loop() {
}


void TaskWifi(void *pvParameters) {

  for (;;) {
    webSocket.loop();
    static uint32_t prev_ms = millis();
    if (millis() > (prev_ms + (1000 / (fps+30)))) {
      String url = "{\"id\":\"" + mac_address + "\", \"bone\":\"" + bone + "\", \"x\":" + quat.x + ", \"y\":" + quat.y + ", \"z\":" + quat.z + ", \"w\":" + quat.w + "}";
      Serial.println(url);

      webSocket.sendTXT(url.c_str());
      prev_ms = millis();
    }
    // vTaskDelay(1);  // one tick delay (15ms) in between reads for stability
  }
}

void TaskReadIMU(void *pvParameters) {
  for (;;) {

    if (myIMU.dataAvailable() == true) {
      quat.x = myIMU.getQuatI();
      quat.y = myIMU.getQuatJ();
      quat.z = myIMU.getQuatK();
      quat.w = myIMU.getQuatReal();
    }



    //vTaskDelay(1);  // one tick delay (15ms) in between reads for stability
  }
}
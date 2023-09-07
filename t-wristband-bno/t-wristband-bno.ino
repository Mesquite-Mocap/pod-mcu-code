#define LILYGO_WATCH_2019_WITH_TOUCH
#include <LilyGoWatch.h>
TTGOClass *watch;
TFT_eSPI *tft;
#include <Wire.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <WebSocketsClient.h> 

#include "SparkFun_BNO080_Arduino_Library.h" // Click here to get the library: https://github.com/sparkfun/SparkFun_BNO080_Arduino_Library
#define I2C_BUFFER_LENGTH 128


BNO080 myIMU;

String mac_address;
unsigned long lastTime = 0;
unsigned long timerDelay = 1000/30;

WiFiMulti WiFiMulti;
WebSocketsClient webSocket;


// ID wifi to connect to 
const char* ssid = "mesquiteMocap";
const char* password = "movement";
String serverIP = "mocap.local";
int sensor_clock = 15; // updated clock - double check your soldering 
int sensor_data = 13; // this is from the soldering. double check what you have soldered your data to 

float batt_v;
float quatI, quatJ, quatK, quatReal;

void hexdump(const void *mem, uint32_t len, uint8_t cols = 16) {
  const uint8_t* src = (const uint8_t*) mem;
  Serial.printf("\n[HEXDUMP] Address: 0x%08X len: 0x%X (%d)", (ptrdiff_t)src, len, len);
  for(uint32_t i = 0; i < len; i++) {
    if(i % cols == 0) {
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

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:  //when disconnected 
      Serial.printf("[WSc] Disconnected!\n");
      break;
    case WStype_CONNECTED: //when connected 
      Serial.printf("[WSc] Connected to url: %s\n", payload);
      // send message to server when Connected
      webSocket.sendTXT("Connected"); //validation that you've connected
      webSocket.enableHeartbeat(1000, 100, 100);
      break;
    case WStype_TEXT: //when you get a message 
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

void setup() {
    // Initialize the watch
  Serial.begin(115200);

  delay(1000);
  
  Wire.flush();
  //delay(100);
  Wire.begin(sensor_clock, sensor_data);
  // Wire.setClock(1000000);

  myIMU.begin(0x4B, Wire);

  if (myIMU.begin() == false) {
    Serial.println("BNO080 not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
    while (1);
  }

  Wire.setClock(400000);

  myIMU.enableRotationVector(50);
  myIMU.enableAccelerometer(50);
  myIMU.enableGyro(50);
  myIMU.enableMagnetometer(50);

  Serial.println(F("Rotation vector enabled"));
  Serial.println(F("Output in form i, j, k, real, accuracy"));

  delay(500);

  WiFiMulti.addAP(ssid, password);

  Serial.println("Connecting");

  while(WiFiMulti.run() != WL_CONNECTED) {
    delay(100);
    Serial.print(".");
  }
  
  Serial.println("");
  Serial.print("Connected to WiFi network with IP Address: "); //this will be the local IP 
  Serial.println(WiFi.localIP());

  mac_address = WiFi.macAddress();
  Serial.println(mac_address);

  delay(500);
  // server address, port and URL


  webSocket.begin(serverIP, 3000, "/hub");

  // event handler
  webSocket.onEvent(webSocketEvent);

  // use HTTP Basic Authorization this is optional remove if not needed
  // webSocket.setAuthorization("user", "Password");

  // try ever 5000 again if connection has failed
  webSocket.setReconnectInterval(5000);
  webSocket.sendTXT(String(millis()).c_str());
}

void loop() {
  // if ((millis() - lastTime) > timerDelay) {
    webSocket.loop();
    if (myIMU.dataAvailable() == true) {
      quatI = myIMU.getQuatI();
      quatJ = myIMU.getQuatJ();
      quatK = myIMU.getQuatK();
      quatReal = myIMU.getQuatReal();
      
    //   batt_v = analogRead(A0);
    
    // String url = "{\"id\": \"" + mac_address + "\",\"x\":" + quatI + ",\"y\":" + quatJ + ",\"z\":" + quatK +  ",\"w\":" + quatReal + "}";
    // String url = String(mac_address) + " " + String(quatI) + " " + String(quatJ) + " " + String(quatK) +  " " + String(quatReal) + " " + String(batt_v);

    // Serial.println(url);
  }

  if ((millis() - lastTime) > timerDelay) {
    String url = "{\"id\": \"" + mac_address + "\",\"x\":" + quatI + ",\"y\":" + quatJ + ",\"z\":" + quatK +  ",\"w\":" + quatReal + "}";
    Serial.println(url);
    webSocket.sendTXT(url.c_str());
    lastTime = millis();
  }
}
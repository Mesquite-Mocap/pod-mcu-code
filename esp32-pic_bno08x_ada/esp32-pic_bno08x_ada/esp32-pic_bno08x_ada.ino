#include <SparkFun_BNO080_Arduino_Library.h>

#include <Arduino.h>
// This demo explores two reports (SH2_ARVR_STABILIZED_RV and SH2_GYRO_INTEGRATED_RV) both can be used to give 
// quartenion and euler (yaw, pitch roll) angles.  Toggle the FAST_MODE define to see other report.  
// Note sensorValue.status gives calibration accuracy (which improves over time)
#include <Adafruit_BNO08x.h>
#include "ttgo.h"
#include <TFT_eSPI.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <WebSocketsClient.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// For SPI mode, we need a CS pin
#define BNO08X_CS 10
#define BNO08X_INT 9


// #define FAST_MODE

// For SPI mode, we also need a RESET 
//#define BNO08X_RESET 5
// but not for I2C or UART
#define BNO08X_RESET -1

#define I2C_SDA_PIN 15
#define I2C_SCL_PIN 13

TFT_eSPI tft = TFT_eSPI();  // Invoke library, pins defined in User_Setup.h
#define TP_PIN_PIN          33
#define IMU_INT_PIN         38
#define RTC_INT_PIN         34
#define BATT_ADC_PIN        35
#define VBUS_PIN            36
#define TP_PWR_PIN          25
#define LED_PIN             4
#define CHARGE_PIN          32


char buff[256];

struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;

struct quaternion_AHRS {
  float qx;
  float qy;
  float qz;
  float qw;
} quat;

String mac_address;
unsigned long lastTime = 0;
unsigned long timerDelay = 1000/30;

WiFiMulti WiFiMulti;
WebSocketsClient webSocket;


// ID wifi to connect to 
const char* ssid = "mesquiteMocap";
const char* password = "movement";
String serverIP = "mocap.local";

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



Adafruit_BNO08x  bno08x;
sh2_SensorValue_t sensorValue;

#ifdef FAST_MODE
  // Top frequency is reported to be 1000Hz (but freq is somewhat variable)
  sh2_SensorId_t reportType = SH2_GYRO_INTEGRATED_RV;
  long reportIntervalUs = 2000;
#else
  // Top frequency is about 250Hz but this report is more accurate
  sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
  long reportIntervalUs = 5000;
#endif
void setReports(sh2_SensorId_t reportType, long report_interval) {
  Serial.println("Setting desired reports");
  if (! bno08x.enableReport(reportType, report_interval)) {
    Serial.println("Could not enable stabilized remote vector");
  }
}

void setup(void) {

  Serial.begin(115200);

  tft.init();
  tft.setRotation(1);
  tft.setSwapBytes(true);
  tft.pushImage(0, 0,  160, 80, ttgo);

  Wire.begin( I2C_SDA_PIN, I2C_SCL_PIN );
  Wire.setClock(400000);
  delay(2000);
  while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit BNO08x test!");
 
  // Try to initialize!
  if (!bno08x.begin_I2C()) {
  //if (!bno08x.begin_UART(&Serial1)) {  // Requires a device with > 300 byte UART buffer!
  //if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    Serial.println("Failed to find BNO08x chip");
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.drawString("Failed to find BNO08x chip", 20, tft.height() / 2 ); 
    while (1) { delay(10); }
  }
  Serial.println("BNO08x Found!");
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.fillScreen(TFT_BLACK);
  tft.drawString("BNO08x Found!", 20, tft.height() / 2 ); 
  delay(2000);

  setReports(reportType, reportIntervalUs);

  Serial.println("Reading events");
  delay(100);

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

void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, quaternion_AHRS* quaternion, bool degrees = false) {

    float sqr = sq(qr);
    float sqi = sq(qi);
    float sqj = sq(qj);
    float sqk = sq(qk);

    quaternion->qx = qi;
    quaternion->qy = qj;
    quaternion->qz = qk;
    quaternion->qw = qr;

    ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

    if (degrees) {
      ypr->yaw *= RAD_TO_DEG;
      ypr->pitch *= RAD_TO_DEG;
      ypr->roll *= RAD_TO_DEG;
    }
}

void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, quaternion_AHRS* quaternion, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr,quaternion, degrees);
}

void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr, quaternion_AHRS* quaternion, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr,quaternion, degrees);
}

void print_roll_pitch_yaw() {    
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.fillScreen(TFT_BLACK);
    tft.setTextDatum(TL_DATUM);;
    //tft.setTextDatum(TL_DATUM);
    // tft.drawString(mac_address,  0, 0, 0);
    // snprintf(buff, sizeof(buff), "%s", mac_address.c_str());
    // tft.drawString(buff, 0, 0);
    snprintf(buff, sizeof(buff), "--  w   x   y   z");
    tft.drawString(buff, 0, 0);
    snprintf(buff, sizeof(buff), "Q %.2f  %.2f  %.2f  %.2f", quat.qw, quat.qx, quat.qy, quat.qz);
    tft.drawString(buff, 0, 16);
   snprintf(buff, sizeof(buff), "E %.2f  %.2f  %.2f", ypr.yaw, ypr.pitch, ypr.roll);
   tft.drawString(buff, 0, 32);
}

void loop() {

  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports(reportType, reportIntervalUs);
  }
  
  if (bno08x.getSensorEvent(&sensorValue)) {
    // in this demo only one report type will be received depending on FAST_MODE define (above)
    switch (sensorValue.sensorId) {
      case SH2_ARVR_STABILIZED_RV:
        quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, &quat, true);
      case SH2_GYRO_INTEGRATED_RV:
        // faster (more noise?)
        quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, &quat, true);
        break;
    }
    static long last = 0;
    long now = micros();
    // Serial.print(now - last);             Serial.print("\t");
    // last = now;
    // Serial.print(sensorValue.status);     Serial.print("\t");  // This is accuracy in the range of 0 to 3
    // Serial.print(ypr.yaw);                Serial.print("\t");
    // Serial.print(ypr.pitch);              Serial.print("\t");
    // Serial.println(ypr.roll);             Serial.print("\t");
    // Serial.print(quat.qw);                Serial.print("\t");
    // Serial.print(quat.qx);              Serial.print("\t");
    // Serial.print(quat.qy);                Serial.print("\t");
    // Serial.print(quat.qz);              

    static uint32_t prev_ms = millis();
      if (millis() > prev_ms + 1) {
          print_roll_pitch_yaw();
          String url = "{\"id\": \"" + mac_address + "\",\"x\":" + quat.qx + ",\"y\":" + quat.qy + ",\"z\":" + quat.qz +  ",\"w\":" + quat.qw + "}";
          Serial.println(url);
          webSocket.sendTXT(url.c_str());
          prev_ms = millis();
      }
  }

}
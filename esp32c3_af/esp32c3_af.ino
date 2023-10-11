#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <WebSocketsClient.h> 
//#define FAST_MODE

#define I2C_SDA_PIN 18
#define I2C_SCL_PIN 19

String mac_address;
unsigned long lastTime = 0;
unsigned long timerDelay = 1000/30;

WiFiMulti WiFiMulti;
WebSocketsClient webSocket;

// ID wifi to connect to 
const char* ssid = "ame494";
const char* password = "12345678";
String serverIP = "192.168.5.161";

struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;

struct quat_t {
  float real;
  float i;
  float j;
  float k;
} quat;

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


void setup(void) {

  Serial.begin(115200);
  Wire.begin( I2C_SDA_PIN, I2C_SCL_PIN );

  while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit BNO08x test!");

  // Try to initialize!
  if (!bno08x.begin_I2C()) {
  //if (!bno08x.begin_UART(&Serial1)) {  // Requires a device with > 300 byte UART buffer!
  //if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    Serial.println("Failed to find BNO08x chip");
    while (1) { delay(10); }
  }
  Serial.println("BNO08x Found!");


  setReports(reportType, reportIntervalUs);

  Serial.println("Reading events");

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


  webSocket.begin(serverIP, 81, "/");

  // event handler
  webSocket.onEvent(webSocketEvent);

  // use HTTP Basic Authorization this is optional remove if not needed
  // webSocket.setAuthorization("user", "Password");

  // try ever 5000 again if connection has failed
  webSocket.setReconnectInterval(0);
  webSocket.sendTXT(String(millis()).c_str());
}

void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {

    float sqr = sq(qr);
    float sqi = sq(qi);
    float sqj = sq(qj);
    float sqk = sq(qk);


    ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

    if (degrees) {
      ypr->yaw *= RAD_TO_DEG;
      ypr->pitch *= RAD_TO_DEG;
      ypr->roll *= RAD_TO_DEG;
    }
}


void saveQuatA(sh2_RotationVectorWAcc_t* rotational_vector, quat_t* q) {
    q->real = rotational_vector->real;
    q->i = rotational_vector->i;
    q->j = rotational_vector->j;
    q->k = rotational_vector->k;
}

void saveQuatG(sh2_GyroIntegratedRV_t* rotational_vector, quat_t* q) {
    q->real = rotational_vector->real;
    q->i = rotational_vector->i;
    q->j = rotational_vector->j;
    q->k = rotational_vector->k;
}

void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);

}

void loop() {
  webSocket.loop();
  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports(reportType, reportIntervalUs);
  }
  if (bno08x.getSensorEvent(&sensorValue)) {
    // in this demo only one report type will be received depending on FAST_MODE define (above)
    switch (sensorValue.sensorId) {
      case SH2_ARVR_STABILIZED_RV:
        //quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
        saveQuatA(&sensorValue.un.arvrStabilizedRV, &quat);
      case SH2_GYRO_INTEGRATED_RV:
        // faster (more noise?)
        //quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);
        saveQuatG(&sensorValue.un.gyroIntegratedRV, &quat);
        break;
    }
        /*

    static long last = 0;
    long now = micros();
    long x = now - last;
    last = now;
    Serial.print(quat.i);     Serial.print(" ");  // This is accuracy in the range of 0 to 3
    Serial.print(quat.j);                Serial.print(" ");
    Serial.print(quat.k);                Serial.print(" ");
    Serial.print(quat.real);                Serial.print(" ");
    //Serial.print(x);             Serial.print("\t");
    //Serial.print(sensorValue.status);
    Serial.println();
    */
  }
      if ((millis() - lastTime) > timerDelay) {
    webSocket.loop(); // !important

      String url = "{\"id\": \"" + mac_address + "\",\"x\":" + quat.i + ",\"y\":" + quat.j + ",\"z\":" + quat.k +  ",\"w\":" + quat.real + "}";
    Serial.println(url);
    webSocket.sendTXT(url.c_str());
    lastTime = millis();
    }

}

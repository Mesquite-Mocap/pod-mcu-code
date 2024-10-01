#include <Wire.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <WebSocketsClient.h>
#include <ESPmDNS.h>

#include <Adafruit_BNO08x.h> // http://librarymanager/All#Adafruit_BNO08x

// #define FAST_MODE

#define BNO08X_RESET -1


Adafruit_BNO08x  bno08x(BNO08X_RESET);
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

#include "esp_adc_cal.h"
#define BAT_ADC 2


// ID wifi to connect to
const char *ssid = "mesquiteMocap";
const char *password = "movement";
String serverIP = "0.0.0.0"; // placeholder; mDNS will resolve.
String dongleName = "mmdongle";
int sensor_clock = 18;  // updated clock - double check your soldering
int sensor_data = 19;   // this is from the soldering. double check what you have soldered your data to


String mac_address;

WiFiMulti WiFiMulti;
WebSocketsClient webSocket;

int fps = 37;
int port = 80;

float batt_v = 0.0;
float quatI, quatJ, quatK, quatReal;



// Choose only one!
// String bone = "LeftArm";
// String bone = "LeftForeArm";
// String bone = "LeftHand";
// String bone = "LeftUpLeg";
// String bone = "LeftLeg";
// String bone = "RightArm";
// String bone = "RightForeArm";
// String bone = "RightHand";
// String bone = "RightUpLeg";
// String bone = "RightLeg";
 String bone = "Spine";
// String bone = "Head";
// String bone = "Hips";


uint32_t readADC_Cal(int ADC_Raw) {
  esp_adc_cal_characteristics_t adc_chars;

  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
  return (esp_adc_cal_raw_to_voltage(ADC_Raw, &adc_chars));
}

boolean start = false;


struct Quat {
  float x;
  float y;
  float z;
  float w;
} quat;

#define NB_RECS 5


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

  while (mdns_init() != ESP_OK) {
    delay(1000);
    Serial.println("Starting MDNS...");
  }
  

  Wire.flush();
  delay(100);
  Wire.begin(sensor_clock, sensor_data);
  delay(1000);
  if (!bno08x.begin_I2C(0x4B, &Wire, 0)) {
    Serial.println("Failed to find BNO08x chip");
    while (1) { delay(10); }
  }
  Serial.println("BNO08x Found!");
  Serial.println(F("IMU enabled"));
  setReports(reportType, reportIntervalUs);



  IPAddress serverIp;

  while (serverIp.toString() == "0.0.0.0") {
    Serial.println("Resolving host...");
    delay(250);
    serverIp = MDNS.queryHost(dongleName);
  }

  	
  Serial.println(serverIp.toString());

  serverIP = serverIp.toString();


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

  // delay(1000);
  xTaskCreatePinnedToCore(
    TaskReadIMU, "TaskReadIMU", 10000  // Stack size
    ,
    NULL, 1  // Priority
    ,
    NULL, 1);

  batt_v = (readADC_Cal(analogRead(BAT_ADC))) * 2;
}

void loop() {
}

int count = 0;


void TaskWifi(void *pvParameters) {
  for (;;) {
    webSocket.loop();
    static uint32_t prev_ms = millis();

    if (millis() > (prev_ms + (1000 / fps))) {
      String url = "{\"id\":\"" + mac_address + "\", \"bone\":\"" + bone + "\", \"x\":" + quat.x + ", \"y\":" + quat.y + ", \"z\":" + quat.z + ", \"w\":" + quat.w + ", \"batt\":" + (batt_v / 4192) + "}";
      Serial.println(url);

      webSocket.sendTXT(url.c_str());
      prev_ms = millis();

      count++;
      if (count > 150) {
        //Serial.println(count);
        if (quat.x == 0 && quat.y == 0 && quat.z == 0 && quat.w == 0) {
          ESP.restart();
        }
        //checkLocking();
       }
    }
    // vTaskDelay(1);  // one tick delay (15ms) in between reads for stability
  }
}

float ax;
float ay;
float az;

void TaskReadIMU(void *pvParameters) {
  for (;;) {

    static uint32_t prev_ms1 = millis();
    if (millis() > (prev_ms1 + 1000 * fps)) {
      // read battery every minute
      batt_v = (readADC_Cal(analogRead(BAT_ADC))) * 2;
      prev_ms1 = millis();
    }

  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports(reportType, reportIntervalUs);
  }
  
    if (bno08x.getSensorEvent(&sensorValue)) {
      sh2_RotationVectorWAcc_t* rotational_vector = &sensorValue.un.arvrStabilizedRV;
      Serial.println(String(rotational_vector->i) + " " + String(rotational_vector->j) + " " + String(rotational_vector->k) + " " + String(rotational_vector->real));
    
      quat.x = rotational_vector->i;
      quat.y = rotational_vector->j;
      quat.z = rotational_vector->k;
      quat.w = rotational_vector->real;
    }



    //vTaskDelay(1);  // one tick delay (15ms) in between reads for stability
  }
}

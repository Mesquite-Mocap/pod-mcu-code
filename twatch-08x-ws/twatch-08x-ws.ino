#include <WiFi.h>
#include <WiFiMulti.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <WebSocketsClient.h> // https://github.com/Links2004/arduinoWebSockets
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <Wire.h>
#define LILYGO_WATCH_2019_WITH_TOUCH 
#include <LilyGoWatch.h>
#include "SparkFun_BNO080_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_BNO080


WiFiMulti WiFiMulti;
WebSocketsClient webSocket;
TTGOClass *watch;
TFT_eSPI *tft;
BNO080 myIMU;

// ID wifi to connect to 
const char* ssid = "mesquiteMocap";
const char* password = "movement";
String serverIP = "mocap.local";
int sensor_clock = 22; // updated clock - double check your soldering 
int sensor_data = 21; // this is from the soldering. double check what you have soldered your data to 
String mac_address; // identifies each seed


// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastTime = 0;
// Timer set to 10 minutes (600000)
//unsigned long timerDelay = 600000;
// Set timer to 5 seconds (5000)
unsigned long timerDelay = 10;

String response;


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

float batt_v;

/*
 *  void pressed() handles keeping the screen off unless the second button is pressed. 
 */

void pressed()
{
       watch->power->adc1Enable(AXP202_VBUS_VOL_ADC1 | AXP202_VBUS_CUR_ADC1 | AXP202_BATT_CUR_ADC1 | AXP202_BATT_VOL_ADC1, true);
    // get the values
     float vbus_v = watch->power->getVbusVoltage();
    float vbus_c = watch->power->getVbusCurrent();
     batt_v = watch->power->getBattVoltage();
    int per = watch->power->getBattPercentage();
         Serial.println(per);

      watch->setBrightness(100);
}

void released()
{
        watch->setBrightness(0);
}


void setup() {
  // Initialize the watch 
  Serial.begin(115200);

      // Get TTGOClass instance
//   watch = TTGOClass::getWatch();

//   // Initialize the hardware, the BMA423 sensor has been initialized internally
//   watch->begin();

//   // Turn on the backlight
//   watch->openBL();


//   //Receive objects for easy writing
//   tft = watch->tft;


// // This intiliazes the button press handler that we defined earlier 
//    watch->button->setPressedHandler(pressed);
//    watch->button->setReleasedHandler(released);

  delay(1000); //  Wait for BNO to boot
  // Start i2c and BNO080
  Wire.flush();   // Reset I2C
  delay(100);
  Wire.begin(sensor_clock, sensor_data);
  myIMU.begin(0x4B, Wire);

// If there is something wrong with the IMU information, check your soldering 
  if (myIMU.begin() == false) {
    Serial.println("BNO080 not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
    while (1);
  }

  Wire.setClock(400000); //Increase I2C data rate to 400kHz

  // This is for the IMU information 

  myIMU.enableRotationVector(50); //Send data update every 50ms

  Serial.println(F("Rotation vector enabled"));
  Serial.println(F("Output in form i, j, k, real, accuracy"));

  delay(500);
  
  WiFiMulti.addAP(ssid, password);

  // Connect to the WiFi 
  
  Serial.println("Connecting");

  while(WiFiMulti.run() != WL_CONNECTED) {
    delay(100);
    Serial.print(".");
  }
  
  Serial.println("");
  Serial.print("Connected to WiFi network with IP Address: "); //this will be the local IP 
  Serial.println(WiFi.localIP());
  Serial.println("Timer set to 5 seconds (timerDelay variable), it will take 5 seconds before publishing the first reading.");

// The MAC address will identify each individual device 
  mac_address = WiFi.macAddress();
  Serial.println(mac_address);
    // Some display settings
  // tft->setTextColor(random(0xFFFF));
  // String t = sensor_clock + " : " + sensor_data;
  // tft->drawString(mac_address,  5, 5, 4);
  // tft->setTextFont(4);
  // tft->setTextColor(TFT_WHITE, TFT_BLACK);

    
  delay(500);
  // server address, port and URL


  webSocket.begin(serverIP, 3000, "/");

  // event handler
  webSocket.onEvent(webSocketEvent);

  // use HTTP Basic Authorization this is optional remove if not needed
  // webSocket.setAuthorization("user", "Password");

  // try ever 5000 again if connection has failed
  webSocket.setReconnectInterval(5000);
  webSocket.sendTXT(String(millis()).c_str());
  // watch->setBrightness(0);

  xTaskCreatePinnedToCore(
    TaskBluetooth
    ,  "TaskBluetooth"   // A name just for humans
    ,  10000  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL 
    ,  0);

  xTaskCreatePinnedToCore(
    TaskRead
    ,  "TaskRead"
    ,  10000  // Stack size
    ,  NULL
    ,  1  // Priority
    ,  NULL 
    ,  1);
}

void loop() {
//     // watch->button->loop();
//     webSocket.loop();
//  if ((millis() - lastTime) > timerDelay) {
//     //Check WiFi connection status
//     if(WiFi.status()== WL_CONNECTED){


//   if (myIMU.dataAvailable() == true)
//   {
//     float quatI = myIMU.getQuatI();
//     float quatJ = myIMU.getQuatJ();
//     float quatK = myIMU.getQuatK();
//     float quatReal = myIMU.getQuatReal();
//     float quatRadianAccuracy = myIMU.getQuatRadianAccuracy();

// /*
//     Serial.print(quatI, 2);
//     Serial.print(F(" "));
//     Serial.print(quatK, 2);
//     Serial.print(F(" "));
//     Serial.print(quatJ, 2);
//     Serial.print(F(" "));
//     Serial.println(quatReal, 2);
//     */

//      tft->fillRect(98, 100, 70, 85, TFT_BLACK);
//       tft->setCursor(80, 60);
//       tft->print("Pins: "); tft->print(sensor_clock);tft->print(",");tft->print(sensor_data);
//       tft->setCursor(80, 90);
//       tft->print("X:"); tft->println(quatI);
//       tft->setCursor(80, 120);
//       tft->print("Y:"); tft->println(quatJ);
//       tft->setCursor(80, 150);
//       tft->print("Z:"); tft->println(quatK);
//       tft->setCursor(80, 180);
//       tft->print("W:"); tft->println(quatReal);
//       tft->setCursor(80, 210);
//       // tft->print("batt:"); tft->println(batt_v);

//       //send to server 

//       String url = "{\"id\": \"" + mac_address + "\",\"x\":" + quatI + ",\"y\":" + quatJ + ",\"z\":" + quatK +  ",\"w\":" + quatReal + ",\"batt\":" + batt_v + "}"; 
//       //Serial.println(url);
//       webSocket.sendTXT(url.c_str());

//   }
//     }
//     else {
//       Serial.println("WiFi Disconnected");
//     }
//     lastTime = millis();
//   }
}

void TaskBluetooth(void *pvParameters) {
  webSocket.loop();
  for (;;) {
      String url = String(mac_address) + " " + quatI + " " + quatJ + " " + quatK +  " " + quatReal;
      Serial.println(url);
      webSocket.sendTXT(url.c_str());
  }
}

void TaskRead(void *pvParameters) {
  for (;;) {
    if (myIMU.dataAvailable() == true) {
      quatI = myIMU.getQuatI();
      quatJ = myIMU.getQuatJ();
      quatK = myIMU.getQuatK();
      quatReal = myIMU.getQuatReal();
    }
  }
}
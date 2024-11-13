#include <EEPROM.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <WebSocketsClient.h>
#include <ESPmDNS.h>
#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#define AD0_VAL 0
ICM_20948_I2C myICM; // Otherwise create an ICM_20948_I2C object

#include "Button2.h"
#define BUTTON_PIN  5
Button2 button;

#include "esp_adc_cal.h"
#define BAT_ADC 2


// ID wifi to connect to
const char *ssid = "mesquiteMocap";
const char *password = "movement";
String serverIP = "0.0.0.0"; // placeholder; mDNS will resolve.
String dongleName = "mmdongle";
int sensor_clock = 19;  // updated clock - double check your soldering
int sensor_data = 18;   // this is from the soldering. double check what you have soldered your data to


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


void setupIMU()
{
    Wire.begin(19,18);
  delay(500);
  Wire.setClock(400000);

  //myICM.enableDebugging();

  bool initialized = false;
  while (!initialized)
  {


    myICM.begin(Wire, AD0_VAL);

    Serial.print(F("Initialization of the sensor returned: "));
    Serial.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      Serial.println(F("Trying again..."));
      delay(500);
    }
    else
    {
      initialized = true;
    }
  }

  Serial.println(F("Device connected."));

  bool success = true; // Use success to show if the DMP configuration was successful

  // Initialize the DMP. initializeDMP is a weak function. In this example we overwrite it to change the sample rate (see below)
  success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);

  // DMP sensor options are defined in ICM_20948_DMP.h
  //    INV_ICM20948_SENSOR_ACCELEROMETER               (16-bit accel)
  //    INV_ICM20948_SENSOR_GYROSCOPE                   (16-bit gyro + 32-bit calibrated gyro)
  //    INV_ICM20948_SENSOR_RAW_ACCELEROMETER           (16-bit accel)
  //    INV_ICM20948_SENSOR_RAW_GYROSCOPE               (16-bit gyro + 32-bit calibrated gyro)
  //    INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED (16-bit compass)
  //    INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED      (16-bit gyro)
  //    INV_ICM20948_SENSOR_STEP_DETECTOR               (Pedometer Step Detector)
  //    INV_ICM20948_SENSOR_STEP_COUNTER                (Pedometer Step Detector)
  //    INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR        (32-bit 6-axis quaternion)
  //    INV_ICM20948_SENSOR_ROTATION_VECTOR             (32-bit 9-axis quaternion + heading accuracy)
  //    INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR (32-bit Geomag RV + heading accuracy)
  //    INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD           (32-bit calibrated compass)
  //    INV_ICM20948_SENSOR_GRAVITY                     (32-bit 6-axis quaternion)
  //    INV_ICM2094 
  //    INV_ICM20948_SENSOR_ORIENTATION                 (32-bit 9-axis quaternion + heading accuracy)

  // Enable the DMP orientation sensor
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_LINEAR_ACCELERATION) == ICM_20948_Stat_Ok);

  // Enable any additional sensors / features
  //success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_GYROSCOPE) == ICM_20948_Stat_Ok);
  //success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_ACCELEROMETER) == ICM_20948_Stat_Ok);
  //success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED) == ICM_20948_Stat_Ok);

  // Configuring DMP to output data at multiple ODRs:
  // DMP is capable of outputting multiple sensor data at different rates to FIFO.
  // Setting value can be calculated as follows:
  // Value = (DMP running rate / ODR ) - 1
  // E.g. For a 5Hz ODR rate when DMP is running at 55Hz, value = (55/5) - 1 = 10.
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok); // Set to the maximum
  //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Accel, 0) == ICM_20948_Stat_Ok); // Set to the maximum
  //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro, 0) == ICM_20948_Stat_Ok); // Set to the maximum
  //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro_Calibr, 0) == ICM_20948_Stat_Ok); // Set to the maximum
  //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass, 0) == ICM_20948_Stat_Ok); // Set to the maximum
  //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass_Calibr, 0) == ICM_20948_Stat_Ok); // Set to the maximum

  // Enable the FIFO
  success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);

  // Enable the DMP
  success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);

  // Reset DMP
  success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);

  // Reset FIFO
  success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);

  // Check success
  if (success)
  {
    Serial.println(F("DMP enabled."));
  }
  else
  {
    Serial.println(F("Enable DMP failed!"));
    Serial.println(F("Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h..."));
    while (1)
      ; // Do nothing more
  }



  Serial.println(F("IMU enabled"));
}


void setup() {


  pinMode(3, OUTPUT);
  Serial.begin(115200);
  delay(500);

  digitalWrite(3, HIGH);

  button.begin(BUTTON_PIN);
  button.setLongClickTime(3000);
  Serial.println(" Longpress Time:\t" + String(button.getLongClickTime()) + "ms");
  button.setLongClickHandler(longClick);
  button.setLongClickDetectedHandler(longClickDetected);

  esp_deep_sleep_enable_gpio_wakeup(BIT(5), ESP_GPIO_WAKEUP_GPIO_LOW);

  WiFiMulti.addAP(ssid, password);

  Serial.println("Connecting");

  int start = millis();
  while (WiFiMulti.run() != WL_CONNECTED) {
    delay(100);
    Serial.print(".");
    if(millis() - start > 1000*20){
          esp_deep_sleep_start();
    }
  }

  Serial.println("");
  Serial.print("Connected to WiFi network with IP Address: ");  //this will be the local IP
  Serial.println(WiFi.localIP());

  mac_address = WiFi.macAddress();
  Serial.println(mac_address);
  delay(100);

  int st = millis();
  while (mdns_init() != ESP_OK) {
    delay(1000);
    Serial.println("Starting MDNS...");
    if(millis() - st > 1000*20){
          esp_deep_sleep_start();
    }
  }

  setupIMU();

  IPAddress serverIp;


  int st2 = millis();
  while (serverIp.toString() == "0.0.0.0") {
    Serial.println("Resolving host...");
    delay(250);
    serverIp = MDNS.queryHost(dongleName);
    if(millis() - st2 > 1000*20){
          esp_deep_sleep_start();
    }
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
    button.loop();
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

icm_20948_DMP_data_t data;
  myICM.readDMPdataFromFIFO(&data);

  if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) // Was valid data available?
  {
    //Serial.print(F("Received data! Header: 0x")); // Print the header in HEX so we can see what data is arriving in the FIFO
    //if ( data.header < 0x1000) Serial.print( "0" ); // Pad the zeros
    //if ( data.header < 0x100) Serial.print( "0" );
    //if ( data.header < 0x10) Serial.print( "0" );
    //Serial.println( data.header, HEX );

    if ((data.header & DMP_header_bitmap_Quat6) > 0) // We have asked for GRV data so we should receive Quat6
    {
      // Q0 value is computed from this equation: Q0^2 + Q1^2 + Q2^2 + Q3^2 = 1.
      // In case of drift, the sum will not add to 1, therefore, quaternion data need to be corrected with right bias values.
      // The quaternion data is scaled by 2^30.

      //Serial.printf("Quat6 data is: Q1:%ld Q2:%ld Q3:%ld\r\n", data.Quat6.Data.Q1, data.Quat6.Data.Q2, data.Quat6.Data.Q3);

      // Scale to +/- 1
      double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
      double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
      double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30


      // Convert the quaternions to Euler angles (roll, pitch, yaw)
      // https://en.wikipedia.org/w/index.php?title=Conversion_between_quaternions_and_Euler_angles&section=8#Source_code_2

      double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

      double q2sqr = q2 * q2;

      // roll (x-axis rotation)
      double t0 = +2.0 * (q0 * q1 + q2 * q3);
      double t1 = +1.0 - 2.0 * (q1 * q1 + q2sqr);
      double roll = atan2(t0, t1) * 180.0 / PI;

      // pitch (y-axis rotation)
      double t2 = +2.0 * (q0 * q2 - q3 * q1);
      t2 = t2 > 1.0 ? 1.0 : t2;
      t2 = t2 < -1.0 ? -1.0 : t2;
      double pitch = asin(t2) * 180.0 / PI;

      // yaw (z-axis rotation)
      double t3 = +2.0 * (q0 * q3 + q1 * q2);
      double t4 = +1.0 - 2.0 * (q2sqr + q3 * q3);
      double yaw = atan2(t3, t4) * 180.0 / PI;


      Serial.print(q0, 3);
      Serial.print(" ");
      Serial.print(q1, 3);
      Serial.print(" ");
      Serial.print(q2, 3);
      Serial.print(" ");
      Serial.print(q3, 3);
      Serial.println();

      quat.w = q0;
      quat.x = q1;
      quat.y = q2;
      quat.z = q3;
    }

  }

  if (myICM.status != ICM_20948_Stat_FIFOMoreDataAvail) // If more data is available then we should read it right away - and not delay
  {
    delay(10);
  }

    //vTaskDelay(1);  // one tick delay (15ms) in between reads for stability
  }
}

void longClickDetected(Button2& btn) {
    Serial.print("---------------- long click #");
    Serial.print(btn.getLongClickCount());
    Serial.println(" detected");
      digitalWrite(3, LOW);
}

void longClick(Button2& btn) {
    esp_deep_sleep_start();
}

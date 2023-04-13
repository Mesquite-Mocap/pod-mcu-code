// #include <BLEDevice.h>
// #include <BLEUtils.h>
// #include <BLEServer.h>
// #include <BLE2902.h>
// #include "esp_bt_device.h"

#include <Wire.h>
#include "SparkFun_BNO080_Arduino_Library.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "BluetoothSerial.h"

// #include <BLESerial.h>

#define LILYGO_WATCH_2019_WITH_TOUCH
#include <LilyGoWatch.h>
TTGOClass *watch;
TFT_eSPI *tft;

// BLESerial bleSerial;

// #define bleeSerial bleSerial

BluetoothSerial SerialBT;

BNO080 myIMU;

int sensor_clock = 22;
int sensor_data = 21;

String mac_address;

float batt_v;
float quatI, quatJ, quatK, quatReal;

// define two tasks for Blink & AnalogRead
void TaskBluetooth(void *pvParameters);
void TaskReadMPU(void *pvParameters);

void pressed() {
  watch->power->adc1Enable(AXP202_VBUS_VOL_ADC1 | AXP202_VBUS_CUR_ADC1 | AXP202_BATT_CUR_ADC1 | AXP202_BATT_VOL_ADC1, true);
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


  Wire.flush();
  Wire.begin(sensor_clock, sensor_data);
  myIMU.begin(BNO080_DEFAULT_ADDRESS, Wire);
  watch = TTGOClass::getWatch();

  delay(1000);

  watch->begin();
  watch->openBL();
  tft = watch->tft;
  watch->button->setPressedHandler(pressed);
  watch->button->setReleasedHandler(released);

  if (myIMU.begin() == false) {
    Serial.println("BNO080 not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
    while (1);
  }

  Wire.setClock(400000);

  myIMU.enableRotationVector(10);
  myIMU.enableAccelerometer(10);
  myIMU.enableGyro(10);
  myIMU.enableMagnetometer(10);

  Serial.println(F("Rotation vector enabled"));
  Serial.println(F("Output in form i, j, k, real, accuracy"));

  delay(500);

  Serial.begin(115200);

  delay(1000);

  SerialBT.begin("MM-1");
  // bleeSerial.begin("IoT-Bus Bluetooth Serial"); 
  Serial.println("Bluetooth Serial started");

  // mac_address = BLEDevice::getAddress().toString().c_str();
  mac_address = "08:3a:f2:44:ca:92";

  delay(500);
  Serial.println("Timer set to 5 seconds (timerDelay variable), it will take 5 seconds before publishing the first reading.");

  Serial.println(mac_address);

  delay(1000);

  xTaskCreatePinnedToCore(
    TaskBluetooth
    ,  "TaskBluetooth"   // A name just for humans
    ,  10000  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL 
    ,  0);

  xTaskCreatePinnedToCore(
    TaskReadMPU
    ,  "TaskReadMPU"
    ,  10000  // Stack size
    ,  NULL
    ,  1  // Priority
    ,  NULL 
    ,  1);
}

void loop() {
  
}

void IMU_Show() {\
  if (myIMU.dataAvailable() == true) {
    quatI = myIMU.getQuatI();
    quatJ = myIMU.getQuatJ();
    quatK = myIMU.getQuatK();
    quatReal = myIMU.getQuatReal();

    tft->setTextColor(random(0xFFFF));
    String t = sensor_clock + " : " + sensor_data;
    tft->drawString(mac_address,  5, 5, 4);
    tft->setTextFont(4);
  }
}

void TaskBluetooth(void *pvParameters) {
  for (;;) {
    String url = "{\"id\": \"" + mac_address + "\",\"x\":" + quatI + ",\"y\":" + quatJ + ",\"z\":" + quatK +  ",\"w\":" + quatReal + ",#" + String(millis()) + "}";
    Serial.println(url);

    int len = url.length();
    uint8_t buf[len + 1];
    url.getBytes(buf, len + 1);
    SerialBT.write(buf, len);
    // SerialBT.println(url);
  }
}

void TaskReadMPU(void *pvParameters) {
    for (;;) {
      // static uint32_t prev_ms = millis();
      // if (millis() > (prev_ms + (1000/50))) {
      //   IMU_Show();
      //   prev_ms = millis();
      // }
      IMU_Show();
    // vTaskDelay(1);  // one tick delay (15ms) in between reads for stability
  }
}
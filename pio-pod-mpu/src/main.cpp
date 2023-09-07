#include "MPU9250.h"
#include "ttgo.h"
#include <TFT_eSPI.h>
//#include "pcf8563.h"
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include "esp_bt_device.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>


MPU9250 mpu;
TFT_eSPI tft = TFT_eSPI(); 
//PCF8563_Class rtc; 
boolean start=false;

BLECharacteristic* pCharacteristic;

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
uint32_t targetTime = 0;       // for next 1 second timeout
uint32_t colour = 0;
int vref = 1100;

bool pressed = false;
uint32_t pressedTime = 0;
bool charge_indication = false;

uint8_t hh, mm, ss ;
String mac_address;
int pacnum=0;


#define TP_PIN_PIN          33
#define I2C_SDA_PIN         21
#define I2C_SCL_PIN         22
#define IMU_INT_PIN         38
#define RTC_INT_PIN         34
#define BATT_ADC_PIN        35
#define VBUS_PIN            36
#define TP_PWR_PIN          25
#define LED_PIN             4
#define CHARGE_PIN          32

//maintain compatability with HM-10
#define BLE_NAME "MM" //must match filters name in bluetoothterminal.js- navigator.bluetooth.requestDevice
// BLEUUID  SERVICE_UUID((uint16_t)0x1802); // UART service UUID
// BLEUUID CHARACTERISTIC_UUID ((uint16_t)0x1803);

// define two tasks for Blink & AnalogRead
void TaskBluetooth(void *pvParameters);
void TaskReadMPU(void *pvParameters);

#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

BLEUUID  SERVICE_UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E"); // UART service UUID
BLEUUID CHARACTERISTIC_UUID ("6E400002-B5A3-F393-E0A9-E50E24DCCA9E");
BLEAdvertising *pAdvertising;

class MyCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pChar) {
    std::string value = pChar->getValue();
    Serial.print("Received Value: ");
    Serial.println(value.c_str());
    start=true;
  }
};

class ServerCallbacks : public BLEServerCallbacks {
    void onDisconnect(BLEServer* server) {
      Serial.print("Disconnected");
      pAdvertising->start();
    }
};


void setup() {
    Serial.begin(115200);

    // Serial.println("");
    // Serial.print("Connected to ");
    // Serial.println(ssid);
    // Serial.print("IP address: ");
    // Serial.println(WiFi.localIP());

    // Serial.println(mac_address);
    // mac_address = WiFi.macAddress();

    tft.init();
    tft.setRotation(1);
    tft.setSwapBytes(true);
    tft.pushImage(0, 0,  160, 80, ttgo);

    BLEDevice::init(BLE_NAME);
    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    BLEService *pService = pServer->createService(SERVICE_UUID);

    pCharacteristic = pService->createCharacteristic(
                                          CHARACTERISTIC_UUID,
                                          BLECharacteristic::PROPERTY_READ |
                                          BLECharacteristic::PROPERTY_WRITE |
                                          BLECharacteristic::PROPERTY_NOTIFY
                                        );

    pCharacteristic->setCallbacks(new MyCallbacks());
    
    pCharacteristic->addDescriptor(new BLE2902());

    mac_address = BLEDevice::getAddress().toString().c_str();

    esp_ble_gap_set_device_name(("MM-" + mac_address).c_str());
    esp_bt_dev_set_device_name(("MM-" + mac_address).c_str());

    pService->start();

    pAdvertising = pServer->getAdvertising();
    pAdvertising->start();

    Wire.begin();
    Wire.setClock(400000);
    delay(2000);

    if (!mpu.setup(0x69)) {  // change to your own address
        while (1) {
            Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
        }
    }

    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);

    while(!start) {
      tft.fillScreen(TFT_BLACK);
      snprintf(buff, sizeof(buff), "%s", mac_address.c_str());
      tft.drawString(mac_address.c_str(), 20, tft.height() / 4 );
      tft.drawString("Waiting for calibration", 10, tft.height() / 2 ); 
      // delay(1000);
    }

    // calibrate anytime you want to
    tft.fillScreen(TFT_BLACK);
    tft.drawString("Accel Gyro calibration - 5sec.",  20, tft.height() / 2 );
    Serial.println("Accel Gyro calibration will start in 5sec.");
    tft.fillScreen(TFT_BLACK);
    tft.drawString("Leave on the flat plane",  20, tft.height() / 2 );
    Serial.println("Please leave the device still on the flat plane.");
    mpu.verbose(true);
    delay(5000);
    mpu.calibrateAccelGyro();

    tft.fillScreen(TFT_BLACK);
    tft.drawString("Mag calibration in 5sec",  20, tft.height() / 2 );
    Serial.println("Mag calibration will start in 5sec.");
    tft.fillScreen(TFT_BLACK);
    tft.drawString("Wave device in eight",  20, tft.height() / 2 );
    Serial.println("Please Wave device in a figure eight until done.");
    delay(5000);
    mpu.calibrateMag();

    mpu.verbose(false);

    pinMode(TP_PIN_PIN, INPUT);
    //! Must be set to pull-up output mode in order to wake up in deep sleep mode
    pinMode(TP_PWR_PIN, PULLUP);
    digitalWrite(TP_PWR_PIN, HIGH);

    pinMode(LED_PIN, OUTPUT);

    pinMode(CHARGE_PIN, INPUT_PULLUP);

    attachInterrupt(CHARGE_PIN, [] {
        charge_indication = true;
    }, CHANGE);

    if (digitalRead(CHARGE_PIN) == LOW) {
        charge_indication = true;
    }

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
  if (digitalRead(TP_PIN_PIN) == HIGH) {
    if (!pressed) {
      pressed = true;
      pressedTime = millis();
    }

    if (millis() - pressedTime > 2000) {
      if (digitalRead(TP_PIN_PIN) == HIGH) {
        tft.fillScreen(TFT_BLACK);
        tft.setTextColor(TFT_GREEN, TFT_BLACK);
        tft.setTextDatum(MC_DATUM);
        tft.drawString("Press again to wake up",  tft.width() / 2, tft.height() / 2 );
        // mpu.setSleepEnabled(true);
        mpu.sleep(true);
        Serial.println("Go to Sleep");
        delay(3000);
        //tft.writecommand(ST7735_SLPIN);
        //tft.writecommand(ST7735_DISPOFF);
        esp_sleep_enable_ext1_wakeup(GPIO_SEL_33, ESP_EXT1_WAKEUP_ANY_HIGH);
        esp_deep_sleep_start();

      } else {
        tft.setTextColor(TFT_GREEN, TFT_BLACK);
        tft.setTextDatum(MC_DATUM);
        tft.drawString("Hold for 2 seconds",  tft.width() / 2, tft.height() / 2 );
      }
    }
  } else {
    pressed = false;
  }
}

// void loop() {
//   if (digitalRead(TP_PIN_PIN) == HIGH) {
//       if (!pressed) {
//         initial = 1;
//         targetTime = millis() + 1000;
//         tft.fillScreen(TFT_BLACK);
//         omm = 99;
//         func_select = func_select + 1 > 2 ? 0 : func_select + 1;
//         digitalWrite(LED_PIN, HIGH);
//         delay(100);
//         digitalWrite(LED_PIN, LOW);
//         pressed = true;
//         pressedTime = millis();
//       } else {
//         if (millis() - pressedTime > 3000) {
//             tft.setTextColor(TFT_GREEN, TFT_BLACK);
//             tft.setTextDatum(MC_DATUM);
//             tft.drawString("Press again to wake up",  tft.width() / 2, tft.height() / 2 );
//             // mpu.setSleepEnabled(true);
//             mpu.sleep(true);
//             Serial.println("Go to Sleep");
//             delay(3000);
//             tft.writecommand(ST7735_SLPIN);
//             tft.writecommand(ST7735_DISPOFF);
//             esp_sleep_enable_ext1_wakeup(GPIO_SEL_33, ESP_EXT1_WAKEUP_ANY_HIGH);
//             esp_deep_sleep_start();
//         }
//       }
//   } else {
//     pressed = false;
//   }

//   switch (func_select) {
//   case 0: {
//       // IMU_Show();

//       // String url = String(pacnum) + "," + mpu.getGyroX() + "," + mpu.getGyroY() + "," + mpu.getGyroZ() + "," + mpu.getAccX() + "," + mpu.getAccY() + "," + mpu.getAccZ() + "," + mpu.getMagX() + "," + mpu.getMagY() + "," + mpu.getMagZ() + "," + quat.x + "," + quat.y +  "," + quat.z +  "," + quat.w;
//       // pacnum++;
      
//       // String url = "{\"id\": \"" + mac_address + "\",\"x\":" + quat.x + ",\"y\":" + quat.y + ",\"z\":" + quat.z +  ",\"w\":" + quat.w + "}";
//       // String url = String(quat.w) + " " + quat.x + " " + quat.y +  " " + quat.z;
//       // String url = "{\"id\": \"" + mac_address + "\",\"x\":" + quat.x + ",\"y\":" + quat.y + ",\"z\":" + quat.z +  ",\"w\":" + quat.w + "}";
//       // Serial.println(url);

//       // pCharacteristic->setValue(url.c_str());
//       // pCharacteristic->notify();
//   }
//       break;
//   case 1: {
//       tft.setTextColor(TFT_GREEN, TFT_BLACK);
//       tft.setTextDatum(MC_DATUM);
//       tft.drawString("Press again to wake up",  tft.width() / 2, tft.height() / 2 );
//       // mpu.setSleepEnabled(true);
//       mpu.sleep(true);
//       Serial.println("Go to Sleep");
//       delay(3000);
//       tft.writecommand(ST7735_SLPIN);
//       tft.writecommand(ST7735_DISPOFF);
//       esp_sleep_enable_ext1_wakeup(GPIO_SEL_33, ESP_EXT1_WAKEUP_ANY_HIGH);
//       esp_deep_sleep_start();
//   }
//       break;
//   default:
//       break;
//   }
// }


void print_roll_pitch_yaw() {    
    //tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.fillScreen(TFT_BLACK);
    //tft.setTextDatum(TL_DATUM);
    // tft.drawString(mac_address,  0, 0, 0);
    snprintf(buff, sizeof(buff), "%s", mac_address.c_str());
    tft.drawString(buff, 0, 0);
    snprintf(buff, sizeof(buff), "--  w   x   y   z");
    tft.drawString(buff, 0, 8);
    snprintf(buff, sizeof(buff), "Q %.2f  %.2f  %.2f  %.2f", quat.w, quat.x, quat.y, quat.z);
    tft.drawString(buff, 0, 16);
   snprintf(buff, sizeof(buff), "E %.2f  %.2f  %.2f", euler.x, euler.y, euler.z);
   tft.drawString(buff, 0, 32);
}

void print_calibration() {
    Serial.println("< calibration parameters >");
    Serial.println("accel bias [g]: ");
    Serial.print(mpu.getAccBiasX() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasY() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasZ() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.println();
    Serial.println("gyro bias [deg/s]: ");
    Serial.print(mpu.getGyroBiasX() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasY() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasZ() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.println();
    Serial.println("mag bias [mG]: ");
    Serial.print(mpu.getMagBiasX());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasY());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasZ());
    Serial.println();
    Serial.println("mag scale []: ");
    Serial.print(mpu.getMagScaleX());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleY());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleZ());
    Serial.println();
}


void IMU_Show() {
  if (mpu.update()) {
    // static uint32_t prev_ms = millis();
    // if (millis() > prev_ms + 50) {
    //     print_roll_pitch_yaw();
    //     prev_ms = millis();
    // }
    print_roll_pitch_yaw();
    quat.x = mpu.getQuaternionX();
    quat.y = mpu.getQuaternionY();
    quat.z = mpu.getQuaternionZ();
    quat.w = mpu.getQuaternionW();
    euler.x = mpu.getEulerX();
    euler.y = mpu.getEulerY();
    euler.z = mpu.getEulerZ();
  }
}

void TaskBluetooth(void *pvParameters) {
  for (;;) {
    String url = "{\"id\": \"" + mac_address + "\",\"x\":" + quat.x + ",\"y\":" + quat.y + ",\"z\":" + quat.z +  ",\"w\":" + quat.w + "}";
    // String url = String(mpu.getGyroX()) + "," + mpu.getGyroY() + "," + mpu.getGyroZ() + "," + mpu.getAccX() + "," + mpu.getAccY() + "," + mpu.getAccZ() + "," + mpu.getMagX() + "," + mpu.getMagY() + "," + mpu.getMagZ();
    Serial.println(url);

    pCharacteristic->setValue(url.c_str());
    pCharacteristic->notify();
    // vTaskDelay(1);  // one tick delay (15ms) in between reads for stability
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
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include "esp_bt_device.h"

#define LILYGO_WATCH_2019_WITH_TOUCH
#include <LilyGoWatch.h>
TTGOClass *watch;
TFT_eSPI *tft;


#include <Wire.h>

#include "SparkFun_BNO080_Arduino_Library.h" // Click here to get the library: https://github.com/sparkfun/SparkFun_BNO080_Arduino_Library
//#define I2C_BUFFER_LENGTH 128


BNO080 myIMU;

int sensor_clock = 22; // updated clock - double check your soldering
int sensor_data = 21; // this is from the soldering. double check what you have soldered your data to

String mac_address;


BLEUUID  SERVICE_UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E"); // UART service UUID
BLEUUID CHARACTERISTIC_UUID ("6E400002-B5A3-F393-E0A9-E50E24DCCA9E");
BLEAdvertising *pAdvertising;
BLECharacteristic* pCharacteristic;
#define BLE_NAME "MM"

class MyCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pChar) {
    std::string value = pChar->getValue();
    Serial.print("Received Value: ");
    Serial.println(value.c_str());
  }
};

class ServerCallbacks : public BLEServerCallbacks {
    void onDisconnect(BLEServer* server) {
      Serial.print("Disconnected");
      pAdvertising->start();
    }
};


float batt_v;


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

  delay(1000);
  
  Wire.flush();
  //delay(100);
  Wire.begin(sensor_clock, sensor_data);
  myIMU.begin(BNO080_DEFAULT_ADDRESS, Wire);

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

  // Initialize the watch
  Serial.begin(115200);

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

  delay(500);
  Serial.println("Timer set to 5 seconds (timerDelay variable), it will take 5 seconds before publishing the first reading.");

  Serial.println(mac_address);
  
  delay(1000);
}

void loop() {
  if (myIMU.dataAvailable() == true)
  {
    float quatI = myIMU.getQuatI();
    float quatJ = myIMU.getQuatJ();
    float quatK = myIMU.getQuatK();
    float quatReal = myIMU.getQuatReal();

    String url = "{\"id\": \"" + mac_address + "\",\"x\":" + quatI + ",\"y\":" + quatJ + ",\"z\":" + quatK +  ",\"w\":" + quatReal + "}";

    Serial.println(url);
    pCharacteristic->setValue(url.c_str());
    pCharacteristic->notify();
  }
}

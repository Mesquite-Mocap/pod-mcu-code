#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include "esp_bt_device.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "SparkFun_BNO080_Arduino_Library.h"
#define I2C_BUFFER_LENGTH 128

#define DEVICE1_ADDRESS "08:3a:f2:44:ca:8e"

BLEClient* pClient;
BLERemoteCharacteristic* pCharacteristicClient;

BNO080 myIMU;

BLEUUID  SERVICE_UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
BLEUUID CHARACTERISTIC_UUID("6E400002-B5A3-F393-E0A9-E50E24DCCA9E");
BLEAdvertising *pAdvertising;
BLECharacteristic* pCharacteristic;
#define BLE_NAME "MM"
String mac_address;

int sensor_clock = 22;
int sensor_data = 21;

void TaskBluetooth(void *pvParameters);
void TaskRead(void *pvParameters);

float quatI, quatJ, quatK, quatReal;
String receivedData = "";

// void onNotify(BLERemoteCharacteristic* pc, uint8_t* pData, size_t length, bool isNotify) {
//   Serial.println(length);
//   std::string str((char*)pData, length);
//   Serial.write(str.c_str());
//   pCharacteristic->setValue(str.c_str());
//   pCharacteristic->notify();
// }

// void onNotify(BLERemoteCharacteristic* pc, uint8_t* pData, size_t length, bool isNotify) {
//   std::string str((char*)pData, length);
//   receivedData += String(str.c_str());

//   // Check for the end of the message (e.g., using a special character or a fixed length)
//   if (receivedData.endsWith("\n")) {
//     Serial.println(receivedData);

//     pCharacteristic->setValue(receivedData.c_str());
//     pCharacteristic->notify();

//     receivedData = ""; // Clear the received data for the next message
//   }
// }

class MyCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pChar) {
    std::string value = pChar->getValue();
    // Serial.print("Received Value: ");
    // Serial.println(value.c_str());
  }
};

class ServerCallbacks : public BLEServerCallbacks {
    void onDisconnect(BLEServer* server) {
      Serial.print("Disconnected");
      pAdvertising->start();
    }
};


void setup() {
  delay(1000);
  Serial.begin(115200);
  
  Wire.flush();
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

  Serial.println(F("Starting Bluetooth Server"));
  
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

  Serial.println(F("Done with Bluetooth Server"));
  Serial.println(F("Starting Bluetooth Client"));

  pClient = BLEDevice::createClient();
  pClient->connect(BLEAddress(DEVICE1_ADDRESS));
  BLERemoteService* pServiceClient = pClient->getService(SERVICE_UUID);
  pCharacteristicClient = pServiceClient->getCharacteristic(CHARACTERISTIC_UUID);
  // pCharacteristicClient->registerForNotify(onNotify);

  Serial.println("Timer set to 5 seconds (timerDelay variable), it will take 5 seconds before publishing the first reading.");
  Serial.println(mac_address);
  Serial.println(F("Done with Bluetooth Client"));

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
  
}

void TaskBluetooth(void *pvParameters) {

  for (;;) {
      std::string value = pCharacteristicClient->readValue();
      // String stringValue = String(value.c_str());

      Serial.println(String(value.c_str()));
      pCharacteristic->setValue(value.c_str());
      pCharacteristic->notify();

      String url = String(mac_address) + " " + quatI + " " + quatJ + " " + quatK +  " " + quatReal;

      Serial.println(url);
      pCharacteristic->setValue(url.c_str());
      pCharacteristic->notify();
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
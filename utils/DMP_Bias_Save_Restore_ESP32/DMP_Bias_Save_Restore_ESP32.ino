#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#define AD0_VAL 0

ICM_20948_I2C myICM; // Otherwise create an ICM_20948_I2C object

#include <EEPROM.h>

// Define a storage struct for the biases. Include a non-zero header and a simple checksum
struct biasStore
{
  int32_t header = 0x42;
  int32_t biasGyroX = 0;
  int32_t biasGyroY = 0;
  int32_t biasGyroZ = 0;
  int32_t biasAccelX = 0;
  int32_t biasAccelY = 0;
  int32_t biasAccelZ = 0;
  int32_t biasCPassX = 0;
  int32_t biasCPassY = 0;
  int32_t biasCPassZ = 0;
  int32_t sum = 0;
};

void updateBiasStoreSum(biasStore *store) // Update the bias store checksum
{
  int32_t sum = store->header;
  sum += store->biasGyroX;
  sum += store->biasGyroY;
  sum += store->biasGyroZ;
  sum += store->biasAccelX;
  sum += store->biasAccelY;
  sum += store->biasAccelZ;
  sum += store->biasCPassX;
  sum += store->biasCPassY;
  sum += store->biasCPassZ;
  store->sum = sum;
}

bool isBiasStoreValid(biasStore *store) // Returns true if the header and checksum are valid
{
  int32_t sum = store->header;

  if (sum != 0x42)
    return false;

  sum += store->biasGyroX;
  sum += store->biasGyroY;
  sum += store->biasGyroZ;
  sum += store->biasAccelX;
  sum += store->biasAccelY;
  sum += store->biasAccelZ;
  sum += store->biasCPassX;
  sum += store->biasCPassY;
  sum += store->biasCPassZ;

  return (store->sum == sum);
}

void printBiases(biasStore *store)
{
  Serial.print(F("Gyro X: "));
  Serial.print(store->biasGyroX);
  Serial.print(F(" Gyro Y: "));
  Serial.print(store->biasGyroY);
  Serial.print(F(" Gyro Z: "));
  Serial.println(store->biasGyroZ);
  Serial.print(F("Accel X: "));
  Serial.print(store->biasAccelX);
  Serial.print(F(" Accel Y: "));
  Serial.print(store->biasAccelY);
  Serial.print(F(" Accel Z: "));
  Serial.println(store->biasAccelZ);
  Serial.print(F("CPass X: "));
  Serial.print(store->biasCPassX);
  Serial.print(F(" CPass Y: "));
  Serial.print(store->biasCPassY);
  Serial.print(F(" CPass Z: "));
  Serial.println(store->biasCPassZ);

}

void setup()
{

  delay(1000);

  Serial.begin(115200); // Start the serial console
  Serial.println(F("ICM-20948 Example"));


  Wire.begin(19,18);
  delay(500);
  Wire.setClock(400000);

  //myICM.enableDebugging();

  bool initialized = false;
  while (!initialized)
  {

    // Initialize the ICM-20948
    // If the DMP is enabled, .begin performs a minimal startup. We need to configure the sample mode etc. manually.
#ifdef USE_SPI
    myICM.begin(CS_PIN, SPI_PORT);
#else
    myICM.begin(Wire, AD0_VAL);
#endif

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
  //    INV_ICM20948_SENSOR_LINEAR_ACCELERATION         (16-bit accel + 32-bit 6-axis quaternion)
  //    INV_ICM20948_SENSOR_ORIENTATION                 (32-bit 9-axis quaternion + heading accuracy)

  // Enable the DMP orientation sensor
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION) == ICM_20948_Stat_Ok);

  // Enable any additional sensors / features
  //success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_GYROSCOPE) == ICM_20948_Stat_Ok);
  //success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_ACCELEROMETER) == ICM_20948_Stat_Ok);
  //success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED) == ICM_20948_Stat_Ok);

  // Configuring DMP to output data at multiple ODRs:
  // DMP is capable of outputting multiple sensor data at different rates to FIFO.
  // Setting value can be calculated as follows:
  // Value = (DMP running rate / ODR ) - 1
  // E.g. For a 5Hz ODR rate when DMP is running at 55Hz, value = (55/5) - 1 = 10.
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat9, 0) == ICM_20948_Stat_Ok); // Set to the maximum
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

  // Read existing biases from EEPROM
  if (!EEPROM.begin(128)) // Allocate 128 Bytes for EEPROM storage. ESP32 needs this.
  {
    Serial.println(F("EEPROM.begin failed! You will not be able to save the biases..."));
  }

  biasStore store;

  EEPROM.get(0, store); // Read existing EEPROM, starting at address 0
  if (isBiasStoreValid(&store))
  {
    Serial.println(F("Bias data in EEPROM is valid. Restoring it..."));
    success &= (myICM.setBiasGyroX(store.biasGyroX) == ICM_20948_Stat_Ok);
    success &= (myICM.setBiasGyroY(store.biasGyroY) == ICM_20948_Stat_Ok);
    success &= (myICM.setBiasGyroZ(store.biasGyroZ) == ICM_20948_Stat_Ok);
    success &= (myICM.setBiasAccelX(store.biasAccelX) == ICM_20948_Stat_Ok);
    success &= (myICM.setBiasAccelY(store.biasAccelY) == ICM_20948_Stat_Ok);
    success &= (myICM.setBiasAccelZ(store.biasAccelZ) == ICM_20948_Stat_Ok);
    success &= (myICM.setBiasCPassX(store.biasCPassX) == ICM_20948_Stat_Ok);
    success &= (myICM.setBiasCPassY(store.biasCPassY) == ICM_20948_Stat_Ok);
    success &= (myICM.setBiasCPassZ(store.biasCPassZ) == ICM_20948_Stat_Ok);

    if (success)
    {
      Serial.println(F("Biases restored."));
      printBiases(&store);
    }
    else
      Serial.println(F("Bias restore failed!"));
  }

  Serial.println(F("The biases will be saved in two minutes."));
  Serial.println(F("Before then:"));
  Serial.println(F("* Rotate the sensor around all three axes"));
  Serial.println(F("* Hold the sensor stationary in all six orientations for a few seconds"));
}

void loop()
{
  static unsigned long startTime = millis(); // Save the biases when the code has been running for two minutes
  static bool biasesStored = false;
  
  icm_20948_DMP_data_t data;
  myICM.readDMPdataFromFIFO(&data);

  if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) // Was valid data available?
  {
    if ((data.header & DMP_header_bitmap_Quat9) > 0) // We have asked for orientation data so we should receive Quat9
    {
      // Scale to +/- 1
      double q1 = ((double)data.Quat9.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
      double q2 = ((double)data.Quat9.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
      double q3 = ((double)data.Quat9.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30

      double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

      double qw = q0; // See issue #145 - thank you @Gord1
      double qx = q2;
      double qy = q1;
      double qz = -q3;

      // roll (x-axis rotation)
      double t0 = +2.0 * (qw * qx + qy * qz);
      double t1 = +1.0 - 2.0 * (qx * qx + qy * qy);
      double roll = atan2(t0, t1) * 180.0 / PI;

      // pitch (y-axis rotation)
      double t2 = +2.0 * (qw * qy - qx * qz);
      t2 = t2 > 1.0 ? 1.0 : t2;
      t2 = t2 < -1.0 ? -1.0 : t2;
      double pitch = asin(t2) * 180.0 / PI;

      // yaw (z-axis rotation)
      double t3 = +2.0 * (qw * qz + qx * qy);
      double t4 = +1.0 - 2.0 * (qy * qy + qz * qz);
      double yaw = atan2(t3, t4) * 180.0 / PI;

/*
      Serial.print(F("Roll: "));
      Serial.print(roll, 1);
      Serial.print(F("\tPitch: "));
      Serial.print(pitch, 1);
      Serial.print(F("\tYaw: "));
      Serial.println(yaw, 1);
      */
      Serial.print(qw, 3);
      Serial.print(" ");
      Serial.print(qx, 3);
      Serial.print(" ");
      Serial.print(qy, 3);
      Serial.print(" ");
      Serial.print(qz, 3);
      Serial.println();

    }
  }

  if (myICM.status != ICM_20948_Stat_FIFOMoreDataAvail) // If more data is available then we should read it right away - and not delay
  {
    if (!biasesStored) // Should we store the biases?
    {
      if (millis() > (startTime + 1*60*1000)) // Is it time to store the biases?
      {
        Serial.println(F("\r\n\r\n\r\nSaving bias data..."));

        biasStore store;
          
        bool success = (myICM.getBiasGyroX(&store.biasGyroX) == ICM_20948_Stat_Ok);
        success &= (myICM.getBiasGyroY(&store.biasGyroY) == ICM_20948_Stat_Ok);
        success &= (myICM.getBiasGyroZ(&store.biasGyroZ) == ICM_20948_Stat_Ok);
        success &= (myICM.getBiasAccelX(&store.biasAccelX) == ICM_20948_Stat_Ok);
        success &= (myICM.getBiasAccelY(&store.biasAccelY) == ICM_20948_Stat_Ok);
        success &= (myICM.getBiasAccelZ(&store.biasAccelZ) == ICM_20948_Stat_Ok);
        success &= (myICM.getBiasCPassX(&store.biasCPassX) == ICM_20948_Stat_Ok);
        success &= (myICM.getBiasCPassY(&store.biasCPassY) == ICM_20948_Stat_Ok);
        success &= (myICM.getBiasCPassZ(&store.biasCPassZ) == ICM_20948_Stat_Ok);

        updateBiasStoreSum(&store);
      
        if (success)
        {
          biasesStored = true; // Only attempt this once
        
          EEPROM.put(0, store); // Write biases to EEPROM, starting at address 0
          EEPROM.commit(); // ESP32/SAMD/STM32 needs this
  
          EEPROM.get(0, store); // Read existing EEPROM, starting at address 0
          if (isBiasStoreValid(&store))
          {
            Serial.println(F("Biases stored."));
            printBiases(&store);
            Serial.println(F("\r\n\r\n\r\n"));
          }
          else
            Serial.println(F("Bias store failed!\r\n\r\n\r\n"));
        }
        else
        {
          Serial.println(F("Bias read failed!\r\n\r\n\r\n"));
        }
      }
    }
      
    //delay(10);
  }
}

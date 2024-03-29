#include <Arduino.h>
// This demo explores two reports (SH2_ARVR_STABILIZED_RV and SH2_GYRO_INTEGRATED_RV) both can be used to give 
// quartenion and euler (yaw, pitch roll) angles.  Toggle the FAST_MODE define to see other report.  
// Note sensorValue.status gives calibration accuracy (which improves over time)
#include <Adafruit_BNO08x.h>

// For SPI mode, we need a CS pin
#define BNO08X_CS 10
#define BNO08X_INT 9


// #define FAST_MODE

// For SPI mode, we also need a RESET 
//#define BNO08X_RESET 5
// but not for I2C or UART
#define BNO08X_RESET -1

#define I2C_SDA_PIN 19
#define I2C_SCL_PIN 18
#define I2C_ADDR 0x4A



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

void setup(void) {

  Serial.begin(115200);
  Wire.begin( I2C_SDA_PIN, I2C_SCL_PIN );

  while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit BNO08x test!");

  // Try to initialize!
  if (!bno08x.begin_I2C(I2C_ADDR, &Wire)) {
  //if (!bno08x.begin_UART(&Serial1)) {  // Requires a device with > 300 byte UART buffer!
  //if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    Serial.println("Failed to find BNO08x chip");
    while (1) { delay(10); }
  }
  Serial.println("BNO08x Found!");


  setReports(reportType, reportIntervalUs);

  Serial.println("Reading events");
  delay(100);
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

  
  }

}

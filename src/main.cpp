/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#include <Arduino.h>
#include <vector>

#include <ArduinoJson.h>

bool deviceConnected = false;
bool oldDeviceConnected = false;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// BLE connection parameters
const int MTU_SIZE = 247;
const float MIN_INTERVAL_MS = 50;
const float MAX_INTERVAL_MS = 100;
const int TIMEOUT_MS = 4000;
const int SLAVE_LATENCY = 3;

#ifdef ADAFRUIT
#include <bluefruit.h>

BLEService        service(SERVICE_UUID);
BLECharacteristic characteristic(CHARACTERISTIC_UUID);

void connect_callback(uint16_t conn_handle) {
    Bluefruit.Periph.setConnIntervalMS(MIN_INTERVAL_MS, MAX_INTERVAL_MS);
    Bluefruit.Periph.setConnSlaveLatency(SLAVE_LATENCY);
    Bluefruit.Periph.setConnSupervisionTimeoutMS(TIMEOUT_MS);

    deviceConnected = true;
    Bluefruit.Advertising.stop();
//    Serial.println("Connected!");
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
    deviceConnected = false;
    if (!Bluefruit.Advertising.isRunning())
        Bluefruit.Advertising.start();
//    Serial.println("Disconnected!");
}

#else
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLE2902.h>

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* server, esp_ble_gatts_cb_param_t *param) {
#ifndef NO_SERIAL
        Serial.println("Current connection params\n--------------------------");
        Serial.print("Interval: "); // Default: 60ms
        Serial.print(param->connect.conn_params.interval * 1.25f);
        Serial.println("ms");
        Serial.print("Latency: ");  // Default: 0
        Serial.println(param->connect.conn_params.latency);
        Serial.print("Timeout: ");  // Default: 9600ms
        Serial.print(param->connect.conn_params.timeout * 10);
        Serial.println("ms");
#endif
        pServer->updatePeerMTU(param->connect.conn_id, MTU_SIZE);
        pServer->updateConnParams(
                param->connect.remote_bda,
                (uint16_t)(MIN_INTERVAL_MS / 1.25f),
                (uint16_t)(MAX_INTERVAL_MS / 1.25f), SLAVE_LATENCY, TIMEOUT_MS / 10);

        deviceConnected = true;

//        // Keep advertising for quick reconnection
//        BLEDevice::startAdvertising();
        BLEDevice::stopAdvertising();
    };

    void onDisconnect(BLEServer* server) {
        deviceConnected = false;

        // Start advertising to allow for reconnection
        BLEDevice::startAdvertising();
    }
};
#endif

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
#include "QMC5883LCompass.h"

#include "calibration_data.h"
#include "position_data.h"
#include "TrivialKalmanVector.h"
#include "FilteredMPU.h"
#include "FilteredQMC.h"

// Uncomment to disable low pass and kalman filtering
#define NO_FILTERS

// Delay initializing the filter to omit extreme data readings
#define KALMAN_DELAY 5000
#define KALMAN_RK 4.7e-3
#define KALMAN_QK 5e-4

uint32_t timer = 0;
bool kalmanSet = false;

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#include <math.h>

#ifdef MPU_COUNT
#define MPU_COUNT 0
const int mpuCount = MPU_COUNT;
#else
const int mpuCount = 1;
#endif

#ifdef QMC_COUNT
#define QMC_COUNT 0
const int qmcCount = QMC_COUNT;
#else
const int qmcCount = 0;
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69

FilteredMPU mpuArray[mpuCount] = {
#if MPU_COUNT > 0
    {
        MPU6050(MPU6050_ADDRESS_AD0_LOW, &Wire),
        TrivialKalmanVector<float>(3, KALMAN_RK, KALMAN_QK),
        TrivialKalmanVector<float>(3, KALMAN_RK, KALMAN_QK),
        TrivialKalmanVector<float>(3, KALMAN_RK, KALMAN_QK),
        TrivialKalmanVector<float>(4, KALMAN_RK, KALMAN_QK),
    },
#endif
#if MPU_COUNT > 1
    {
        MPU6050(MPU6050_ADDRESS_AD0_HIGH, &Wire),
        TrivialKalmanVector<float>(3, KALMAN_RK, KALMAN_QK),
        TrivialKalmanVector<float>(3, KALMAN_RK, KALMAN_QK),
        TrivialKalmanVector<float>(3, KALMAN_RK, KALMAN_QK),
        TrivialKalmanVector<float>(4, KALMAN_RK, KALMAN_QK),
    },
#endif
#if MPU_COUNT > 2
    {
        MPU6050(MPU6050_ADDRESS_AD0_LOW, &Wire1),
        TrivialKalmanVector<float>(3, KALMAN_RK, KALMAN_QK),
        TrivialKalmanVector<float>(3, KALMAN_RK, KALMAN_QK),
        TrivialKalmanVector<float>(3, KALMAN_RK, KALMAN_QK),
        TrivialKalmanVector<float>(4, KALMAN_RK, KALMAN_QK),
    },
#endif
#if MPU_COUNT > 3
    {
        MPU6050(MPU6050_ADDRESS_AD0_HIGH, &Wire1),
        TrivialKalmanVector<float>(3, KALMAN_RK, KALMAN_QK),
        TrivialKalmanVector<float>(3, KALMAN_RK, KALMAN_QK),
        TrivialKalmanVector<float>(3, KALMAN_RK, KALMAN_QK),
        TrivialKalmanVector<float>(4, KALMAN_RK, KALMAN_QK),
    },
#endif
};

FilteredQMC qmcArray[qmcCount] = {
#if QMC_COUNT > 0
    {
        QMC5883LCompass(&Wire),
        TrivialKalmanVector<float>(3, KALMAN_RK, KALMAN_QK),
    },
#endif
#if QMC_COUNT > 1
    {
        QMC5883LCompass(&Wire1),
        TrivialKalmanVector<float>(3, KALMAN_RK, KALMAN_QK),
    },
#endif
};

// when defined, no serial connection will be attempted.
#define NO_SERIAL

// uncomment "SERIAL_PRINT_TITLE" if you want to print the values without the titles
//#define SERIAL_PRINT_TITLE

// MPU control/status vars
bool dmpReady[4] = { false, false, false, false };  // set true if DMP init was successful
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
VectorInt16 gyro;
float euler[3];         // [psi, theta, phi]    Euler angle container

void inline SerialPrintTitle(const char* text) {
#ifdef SERIAL_PRINT_TITLE
    Serial.print(text);
    Serial.print("\t");
#endif
}

std::vector<float> inline GetVectorInt16(VectorInt16& v) {
    return {(float)v.x, (float)v.y, (float)v.z};
}

std::vector<float> inline GetQuaternion(Quaternion& q) {
    return {q.w, q.x, q.y, q.z};
}

#define xstr(s) str(s)
#define str(s) #s

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#ifdef SDA0
    Wire.setPins(SDA0, SCL0);
#endif
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

#ifdef TWO_I2C
    Wire1.begin(SDA1, SCL1);
    Wire1.setClock(400000);
#endif

#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
#endif

#ifdef ADAFRUIT
    Bluefruit.begin();
    Bluefruit.setName(xstr(DEVICE_NAME));
    Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values

    Bluefruit.Periph.setConnectCallback(connect_callback);
    Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

    // Initialize service before any characteristics
    service.begin();

    characteristic.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ | CHR_PROPS_WRITE | CHR_PROPS_INDICATE);
    characteristic.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
    characteristic.setMaxLen(MTU_SIZE);
    characteristic.begin();

    // This descriptor is included by default, no need to add it.
//    characteristic.addDescriptor(BLE_UUID_DESCRIPTOR_CLIENT_CHAR_CONFIG, "", 0);

    // Advertising packet
    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addTxPower();

    Bluefruit.Advertising.addService(service);

    Bluefruit.Advertising.addName();


    /* Start Advertising
     * - Enable auto advertising if disconnected
     * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
     * - Timeout for fast mode is 30 seconds
     * - Start(timeout) with timeout = 0 will advertise forever (until connected)
     *
     * For recommended advertising interval
     * https://developer.apple.com/library/content/qa/qa1931/_index.html
     */
//    Bluefruit.Advertising.setStopCallback(adv_stop_callback);
//    Bluefruit.Advertising.restartOnDisconnect(true);
    Bluefruit.Advertising.setInterval(32, 244);    // in units of 0.625 ms
    Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
    Bluefruit.Advertising.start();      // Stop advertising entirely after ADV_TIMEOUT seconds
#else
    BLEDevice::init(xstr(DEVICE_NAME));

    // Create the BLE Server
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    // Create the BLE Service
    BLEService *pService = pServer->createService(SERVICE_UUID);

    // Create a BLE Characteristic
    pCharacteristic = pService->createCharacteristic(
            CHARACTERISTIC_UUID,
            BLECharacteristic::PROPERTY_READ   |
            BLECharacteristic::PROPERTY_WRITE  |
            BLECharacteristic::PROPERTY_NOTIFY |
            BLECharacteristic::PROPERTY_INDICATE
    );

    // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
    // Create a BLE Descriptor
    pCharacteristic->addDescriptor(new BLE2902());

    // Start the service
    pService->start();

    // Start advertising
//    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
//    pAdvertising->addServiceUUID(SERVICE_UUID);
//    pAdvertising->setScanResponse(false);
//    pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
    BLEDevice::startAdvertising();
//    Serial.println("Waiting a client connection to notify...");
#endif

    // initialize serial communication after bluetooth
#ifndef NO_SERIAL
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately
    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.
#endif

#ifndef NO_SERIAL
    // wait for ready
//    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again
#else
    // Delay initialization if not using serial
    delay(500);
#endif

    // Initialize MPUs
    for (int i = 0; i < mpuCount; i++) {
        MPU6050 &mpu = (mpuArray[i].mpu);
        mpu.initialize();

        // return status after device operation (0 = success, !0 = error)
        int devStatus = mpu.dmpInitialize();

        // supply your own gyro offsets here, scaled for min sensitivity
        calibrateMPU(mpu, i);

#ifndef NO_FILTERS
        // Set Low Pass Filter to 10Hz
        mpu.setDLPFMode(MPU6050_DLPF_BW_10);
#endif

        if (devStatus == 0) {
            // Calibration Time: generate offsets and calibrate our MPU6050
//        mpu.CalibrateAccel(6);
//        mpu.CalibrateGyro(6);
//        mpu.PrintActiveOffsets();

            // turn on the DMP, now that it's ready
//            Serial.println(F("Enabling DMP..."));
            mpu.setDMPEnabled(true);

            // set our DMP Ready flag so the main loop() function knows it's okay to use it
//        Serial.println(F("DMP ready! Waiting for first interrupt..."));
            dmpReady[i] = true;
        } else {
            // ERROR!
            // 1 = initial memory load failed
            // 2 = DMP configuration updates failed
            // (if it's going to break, usually the code will be 1)
#ifndef NO_SERIAL
            Serial.print(F("DMP Initialization failed (code "));
            Serial.print(devStatus);
            Serial.println(F(")"));
#endif
        }
    }

    // Initialize QMCs
    for (int i = 0; i < qmcCount; i++) {
        qmcArray[i].qmc.init();

//        qmcArray[i].qmc.calibrate();
//
//        Serial.print("QMC Calibration Offsets: ");
//        Serial.print(qmcArray[i].qmc.getCalibrationOffset(0));
//        Serial.print(", ");
//        Serial.print(qmcArray[i].qmc.getCalibrationOffset(1));
//        Serial.print(", ");
//        Serial.println(qmcArray[i].qmc.getCalibrationOffset(2));
//        Serial.print("QMC Calibration Scales: ");
//        Serial.print(qmcArray[i].qmc.getCalibrationScale(0));
//        Serial.print(", ");
//        Serial.print(qmcArray[i].qmc.getCalibrationScale(1));
//        Serial.print(", ");
//        Serial.println(qmcArray[i].qmc.getCalibrationScale(2));

        calibrateQMC(qmcArray[i].qmc, i);
    }

    // Record starting time to delay Kalman filtering
    timer = millis();
}


JsonDocument mpu_read(FilteredMPU& filtered, SensorType sensors) {
    JsonDocument doc;

    // read a packet from FIFO
    if (filtered.mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet

        // Always calculate quaternion as it is used by the other measurements
        // display quaternion values in easy matrix form: w x y z
        filtered.mpu.dmpGetQuaternion(&q, fifoBuffer);

        auto predictedQuaternion = GetQuaternion(q);
        if (kalmanSet)
            predictedQuaternion = filtered.quat.update(predictedQuaternion);

        if ((sensors & SensorType::Quat) != SensorType::None) {
            doc["q"] = JsonDocument();
            doc["q"].add((int)(predictedQuaternion[0] * 1000));
            doc["q"].add((int)(predictedQuaternion[1] * 1000));
            doc["q"].add((int)(predictedQuaternion[2] * 1000));
            doc["q"].add((int)(predictedQuaternion[3] * 1000));
        }

        if ((sensors & SensorType::Euler) != SensorType::None) {
            // display Euler angles in degrees
            filtered.mpu.dmpGetEuler(euler, &q);

            auto predictedEuler = std::vector<float>(euler, euler + 3);
            if (kalmanSet)
                predictedEuler = filtered.euler.update(predictedEuler);

            // TODO: Figure out what is causing NaN values for Euler Y angle
            doc["e"] = JsonDocument();
            doc["e"].add(isnan(predictedEuler[0]) ? 0 : (int) (predictedEuler[0] * 180 / M_PI));
            doc["e"].add(isnan(predictedEuler[1]) ? 0 : (int) (predictedEuler[1] * 180 / M_PI));
            doc["e"].add(isnan(predictedEuler[2]) ? 0 : (int) (predictedEuler[2] * 180 / M_PI));
        }

        if ((sensors & SensorType::Gyro) != SensorType::None) {
            filtered.mpu.dmpGetGyro(&gyro, fifoBuffer);

            auto predictedGyro = GetVectorInt16(gyro);
            if (kalmanSet)
                predictedGyro = filtered.gyro.update(GetVectorInt16(gyro));

            doc["g"] = JsonDocument();
            doc["g"].add((int) (predictedGyro[0] * 1000));
            doc["g"].add((int) (predictedGyro[1] * 1000));
            doc["g"].add((int) (predictedGyro[2] * 1000));
        }

        if ((sensors & SensorType::Acc) != SensorType::None) {
            // acceleration components with gravity removed and adjusted for the world frame of
            // reference (yaw is relative to initial orientation, since no magnetometer
            // is present in this case).
            filtered.mpu.dmpGetAccel(&aa, fifoBuffer);
            filtered.mpu.dmpGetGravity(&gravity, &q);
            filtered.mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            filtered.mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

            auto predictedAcc = GetVectorInt16(aaWorld);
            if (kalmanSet)
                predictedAcc = filtered.acc.update(GetVectorInt16(aaWorld));

            doc["a"] = JsonDocument();
            doc["a"].add((int) predictedAcc[0]);
            doc["a"].add((int) predictedAcc[1]);
            doc["a"].add((int) predictedAcc[2]);
        }
    }

    return doc;
}

JsonDocument qmc_read(FilteredQMC& filtered) {
    JsonDocument doc;

    filtered.qmc.read();

    auto predictedMag = std::vector<float>{ (float)filtered.qmc.getX(), (float)filtered.qmc.getY(), (float)filtered.qmc.getZ() };
    if (kalmanSet)
        predictedMag = filtered.mag.update(predictedMag);

    doc["m"] = JsonDocument();
    doc["m"].add((int)predictedMag[0]);
    doc["m"].add((int)predictedMag[1]);
    doc["m"].add((int)predictedMag[2]);

    return doc;
}

void reinit_mpu(int i) {
    mpuArray[i].mpu.initialize();
    calibrateMPU(mpuArray[i].mpu, i);
    mpuArray[i].mpu.setDMPEnabled(true);
    dmpReady[i] = true;
}

void reinit_qmc(int i) {
    qmcArray[i].qmc.init();
    calibrateQMC(qmcArray[i].qmc, i);
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

// PLAN: Buffer the data according to the processing capabilities of the mcu
// Send after every x data points, measure average time taken to process those x data points
// We want to synchronize all mcus and still have a good data rate
//
// Once we update the characteristic value, we leave it up for y ms
// The pi should read the value in those y ms, which should be enough for going over all the mcus in one interval
// We're going to have to sync the data once we get it on the central pi, so we should store the initial time of each mcu in the pi
// We can then calculate the time difference between the mcus and the pi, and use that to sync the data
//
// Filtering should be done on the server pi, right before giving the data to the model

// TODO: Check this with different values for each board, and use as a build flag in platformio.ini
#define BLE_LOOP_DELAY 150

#define SENSOR_CHECK_DELAY 2000
static int sensorCheckTimer = 0;

void loop() {
    JsonDocument doc;

    doc["mpu"] = JsonDocument();
    doc["qmc"] = JsonDocument();

    // Delay between each sensor reading
    delay(BLE_LOOP_DELAY);

    if ((millis() - sensorCheckTimer) > SENSOR_CHECK_DELAY) {
        sensorCheckTimer = millis();
        for (int i = 0; i < mpuCount; i++) {
            if (!dmpReady[i] || !mpuArray[i].mpu.testConnection()) {
                reinit_mpu(i);

                // If the MPU was disconnected, then the magnetometer on the same bus probably was too
                int busIndex = i / 2;
                if (qmcCount > busIndex) {
                    reinit_qmc(busIndex);
                }
            }
        }
    }

#ifndef NO_FILTERS
    if (!kalmanSet && (millis() - timer) > KALMAN_DELAY) {
        kalmanSet = true;
    }
#endif

    for (int i = 0; i < mpuCount; i++) {
        if (dmpReady[i]) {
            doc["mpu"].add(mpu_read(mpuArray[i], getSensorsForMPU(i)));
        }
    }

    for (int i = 0; i < qmcCount; i++) {
        if (dmpReady[i]) {
            doc["qmc"].add(qmc_read(qmcArray[i]));
        }
    }

    if (deviceConnected) {
        if (doc.size() != 0) {
            doc["time"] = millis();

            std::string values;
            serializeMsgPack(doc, values);

            size_t size = values.size();
            if (size > 242) {
#ifndef NO_SERIAL
                Serial.println("WARNING: MessagePack size exceeds 242 bytes");
#endif
            }

#ifdef ADAFRUIT
            characteristic.notify(values.c_str(), size);
#else
            pCharacteristic->setValue(values);
            pCharacteristic->notify();
#endif
#ifndef NO_SERIAL
            Serial.println(values.c_str());
#endif
        }
        delay(33); // bluetooth stack will go into congestion, if too many packets are sent, in 6 hours test i was able to go as low as 3ms
    }
    // disconnecting
    if (!deviceConnected && oldDeviceConnected) {
        delay(500); // give the bluetooth stack the chance to get things ready
#ifdef ADAFRUIT
        Bluefruit.Advertising.start();
#else
        pServer->startAdvertising(); // restart advertising
#endif
#ifndef NO_SERIAL
        Serial.println("start advertising");
#endif
        oldDeviceConnected = deviceConnected;
    }
    // connecting
    if (deviceConnected && !oldDeviceConnected) {
        // do stuff here on connecting
        oldDeviceConnected = deviceConnected;
    }
}

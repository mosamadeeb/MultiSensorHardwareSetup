// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2019-07-08 - Added Auto Calibration and offset generator
//		   - and altered FIFO retrieval sequence to avoid using blocking code
//      2016-04-18 - Eliminated a potential infinite loop
//      2013-05-08 - added seamless Fastwire support
//                 - added note about gyro calibration
//      2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility error
//      2012-06-20 - improved FIFO overflow handling and simplified read process
//      2012-06-19 - completely rearranged DMP initialization code and simplification
//      2012-06-13 - pull gyro and accel data from FIFO packet instead of reading directly
//      2012-06-09 - fix broken FIFO read sequence and change interrupt detection to RISING
//      2012-06-05 - add gravity-compensated initial reference frame acceleration output
//                 - add 3D math helper file to DMP6 example sketch
//                 - add Euler output and Yaw/Pitch/Roll output formats
//      2012-06-04 - remove accel offset clearing for better results (thanks Sungon Lee)
//      2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec instead of 250
//      2012-05-30 - basic DMP initialization working

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

#ifdef ADAFRUIT
#include <bluefruit.h>

BLEService        service(SERVICE_UUID);
BLECharacteristic characteristic(CHARACTERISTIC_UUID);

void connect_callback(uint16_t conn_handle) {
    deviceConnected = true;
//    Serial.println("Connected!");
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
    deviceConnected = false;
//    Serial.println("Disconnected!");
}

#else
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLE2902.h>

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* server) {
        deviceConnected = true;
        BLEDevice::startAdvertising();
    };

    void onDisconnect(BLEServer* server) {
        deviceConnected = false;
    }
};
#endif

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
#include "QMC5883LCompass.h"

#include "TrivialKalmanVector.h"

#include "FilteredMPU.h"
#include "FilteredQMC.h"

// Delay initializing the filter to omit extreme data readings
#define KALMAN_DELAY 5000
#define KALMAN_RK 4.7e-3
#define KALMAN_QK 5e-4

unsigned long timer = 0;
bool kalmanSet = false;

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#include <math.h>

#ifdef MPU_COUNT
const int mpuCount = MPU_COUNT;
#else
const int mpuCount = 1;
#endif

#ifdef QMC_COUNT
const int qmcCount = QMC_COUNT;
#else
const int qmcCount = 0;
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69

FilteredMPU mpuArray[mpuCount] = {
    {
        MPU6050(MPU6050_ADDRESS_AD0_LOW, &Wire),
        TrivialKalmanVector<float>(3, KALMAN_RK, KALMAN_QK),
        TrivialKalmanVector<float>(3, KALMAN_RK, KALMAN_QK),
        TrivialKalmanVector<float>(3, KALMAN_RK, KALMAN_QK),
        TrivialKalmanVector<float>(4, KALMAN_RK, KALMAN_QK),
    },
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

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */

/* =========================================================================
   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
   when using Serial.write(buf, len). The Teapot output uses this method.
   The solution requires a modification to the Arduino USBAPI.h file, which
   is fortunately simple, but annoying. This will be fixed in the next IDE
   release. For more info, see these links:

   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
 * ========================================================================= */

// choose sensor number for calibration values
#define SENSOR_2

// when defined, no serial connection will be attempted.
#define NO_SERIAL

// uncomment "SERIAL_PRINT_TITLE" if you want to print the values without the titles
//#define SERIAL_PRINT_TITLE

// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL

//#define OUTPUT_READABLE_GYRO

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
#define OUTPUT_READABLE_WORLDACCEL

// uncomment these to print the respective vectors
// applies for OUTPUT_READABLE_REALACCEL and OUTPUT_READABLE_WORLDACCEL
//#define OUTPUT_REALACC
//#define OUTPUT_ACC
//#define OUTPUT_GRAVITY

//#define OUTPUT_READABLE_MAGNET

bool blinkState = false;
// MPU control/status vars
bool dmpReady[4] = { false, false, false, false };  // set true if DMP init was successful
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
VectorInt16 gyro;
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

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
    Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values

    Bluefruit.Periph.setConnectCallback(connect_callback);
    Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

    // Initialize service before any characteristics
    service.begin();

    characteristic.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ | CHR_PROPS_WRITE | CHR_PROPS_INDICATE);
    characteristic.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
    characteristic.setMaxLen(128);
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
    Bluefruit.Advertising.restartOnDisconnect(true);
    Bluefruit.Advertising.setInterval(32, 244);    // in units of 0.625 ms
    Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
    Bluefruit.Advertising.start();      // Stop advertising entirely after ADV_TIMEOUT seconds
#else
    BLEDevice::init("ESP32-2");

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
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(false);
    pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
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
#endif

    // Initialize MPUs
    for (int i = 0; i < mpuCount; i++) {
        MPU6050 &mpu = (mpuArray[i].mpu);
        mpu.initialize();
        devStatus = mpu.dmpInitialize();

        // supply your own gyro offsets here, scaled for min sensitivity
#ifdef SENSOR_1
        mpu.setXAccelOffset(-909);
        mpu.setYAccelOffset(-2);
        mpu.setZAccelOffset(1062);
        mpu.setXGyroOffset(72);
        mpu.setYGyroOffset(58);
        mpu.setZGyroOffset(-10);
#endif
#ifdef SENSOR_2
        mpu.setXAccelOffset(-3874);
        mpu.setYAccelOffset(13);
        mpu.setZAccelOffset(1058);
        mpu.setXGyroOffset(-5);
        mpu.setYGyroOffset(-17);
        mpu.setZGyroOffset(-24);
#endif
        // Set Low Pass Filter to 10Hz
        mpu.setDLPFMode(MPU6050_DLPF_BW_10);

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

            // get expected DMP packet size for later comparison
            packetSize = mpu.dmpGetFIFOPacketSize();
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
    }

    // Record starting time to delay Kalman filtering
    timer = millis();

#ifdef LED_PIN
    // configure LED for output
    pinMode(LED_PIN, OUTPUT);

    // Turn on LED to indicate that filter is not working
    digitalWrite(LED_PIN, true);
#endif
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

JsonDocument mpu_print(FilteredMPU& filtered) {
    JsonDocument doc;

    // read a packet from FIFO
    if (filtered.mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet

        // display quaternion values in easy matrix form: w x y z
        filtered.mpu.dmpGetQuaternion(&q, fifoBuffer);

        auto predictedQuaternion = GetQuaternion(q);
        if (kalmanSet)
            predictedQuaternion = filtered.quat.update(predictedQuaternion);

        doc["q"] = JsonDocument();
        doc["q"].add(predictedQuaternion[0]);
        doc["q"].add(predictedQuaternion[1]);
        doc["q"].add(predictedQuaternion[2]);
        doc["q"].add(predictedQuaternion[3]);

#ifdef OUTPUT_READABLE_QUATERNION
        SerialPrintTitle("quat");
        Serial.print(round(q.w*1000));
        Serial.print("\t");
        Serial.print(round(q.x*1000));
        Serial.print("\t");
        Serial.print(round(q.y*1000));
        Serial.print("\t");
        Serial.println(round(q.z*1000));
#endif

        // display Euler angles in degrees
        filtered.mpu.dmpGetQuaternion(&q, fifoBuffer);
        filtered.mpu.dmpGetEuler(euler, &q);

        auto predictedEuler = std::vector<float>(euler, euler + 3);
        if (kalmanSet)
            predictedEuler = filtered.euler.update(predictedEuler);

        doc["e"] = JsonDocument();
        doc["e"].add(predictedEuler[0]);
        doc["e"].add(predictedEuler[1]);
        doc["e"].add(predictedEuler[2]);

#ifdef OUTPUT_READABLE_EULER
        SerialPrintTitle("euler");
        Serial.print(round(euler[0] * 180/M_PI));
        Serial.print("\t");
        Serial.print(round(euler[1] * 180/M_PI));
        Serial.print("\t");
        Serial.println(round(euler[2] * 180/M_PI));
#endif

        filtered.mpu.dmpGetGyro(&gyro,fifoBuffer);

        auto predictedGyro = GetVectorInt16(gyro);
        if (kalmanSet)
            predictedGyro = filtered.gyro.update(GetVectorInt16(gyro));

        doc["g"] = JsonDocument();
        doc["g"].add(predictedGyro[0]);
        doc["g"].add(predictedGyro[1]);
        doc["g"].add(predictedGyro[2]);

#ifndef NO_SERIAL
        SerialPrintTitle("AngularVal");
        Serial.print(gyro.x);
        Serial.print("\t");
        Serial.print(gyro.y);
        Serial.print("\t");
        Serial.println(gyro.z);
#endif

        // display initial world-frame acceleration, adjusted to remove gravity
        // and rotated based on known orientation from quaternion
        filtered.mpu.dmpGetQuaternion(&q, fifoBuffer);
        filtered.mpu.dmpGetAccel(&aa, fifoBuffer);
        filtered.mpu.dmpGetGravity(&gravity, &q);
        filtered.mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        filtered.mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

        auto predictedAcc = GetVectorInt16(aaWorld);
        if (kalmanSet)
            predictedAcc = filtered.acc.update(GetVectorInt16(aaWorld));

        doc["a"] = JsonDocument();
        doc["a"].add(predictedAcc[0]);
        doc["a"].add(predictedAcc[1]);
        doc["a"].add(predictedAcc[2]);
    }

    return doc;
}

JsonDocument qmc_print(FilteredQMC& filtered) {
    JsonDocument doc;

    filtered.qmc.read();

    auto predictedMag = std::vector<float>{ (float)filtered.qmc.getX(), (float)filtered.qmc.getY(), (float)filtered.qmc.getZ() };
    if (kalmanSet)
        predictedMag = filtered.mag.update(predictedMag);

    doc["m"] = JsonDocument();
    doc["m"].add(predictedMag[0]);
    doc["m"].add(predictedMag[1]);
    doc["m"].add(predictedMag[2]);

    return doc;
}

void loop() {
    JsonDocument doc;

    doc["mpu"] = JsonDocument();
    JsonArray mpu = doc["mpu"];

    doc["qmc"] = JsonDocument();
    JsonArray qmc = doc["qmc"];

#ifdef NO_SERIAL
    // 33ms delay between each sensor reading
    delay(33);

    if (!kalmanSet && millis() - timer > KALMAN_DELAY) {
        kalmanSet = true;
    }

    for (int i = 0; i < mpuCount; i++) {
        if (dmpReady[i]) {
            mpu.add(mpu_print(mpuArray[i]));
        }
    }

    for (int i = 0; i < qmcCount; i++) {
        if (dmpReady[i]) {
            qmc.add(qmc_print(qmcArray[i]));
        }
    }
#else
    while (Serial.available() <= 0);

    char c = Serial.read();
    if (c == 'g' && dmpReady[0]) {
        doc["mpu1"] = mpu_print(mpuArray[0]);
    } else if (c == 'f' && dmpReady[1]) {
        doc["mpu2"] = mpu_print(mpuArray[1]);
    } else if (c == 'd' && dmpReady[2]) {
        doc["mpu3"] = mpu_print(mpuArray[2]);
    } else if (c == 's' && dmpReady[3]) {
        doc["mpu4"] = mpu_print(mpuArray[3]);
    }
#endif

    if (deviceConnected) {
        if (doc.size() != 0) {
            doc["timestamp"] = millis();

            std::string values;
            serializeMsgPack(doc, values);

            size_t size = values.size();
            if (size > 512) {
#ifndef NO_SERIAL
                Serial.println("WARNING: MessagePack size exceeds 512 bytes");
#endif
            }

#ifdef ADAFRUIT
            characteristic.write(values.c_str(), size);
#else
            pCharacteristic->setValue(values);
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

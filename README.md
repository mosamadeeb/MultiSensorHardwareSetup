# MultiSensorHardwareSetup

Embedded program that collects data from multiple 6+3 axis sensors and sends them over to a local server via Bluetooth LE for ML inference. Designed to be deployed and synchronized on multiple MCUs.

This repository is part of our Capstone project at METU NCC: [Multi Sensor Activity Based Health Monitoring System](https://www.linkedin.com/posts/noor-ul-zain-305144145_happy-to-share-the-successful-completion-activity-7211286326636122112-rPOE)

## Context

The sensors are placed on various locations on the body (arms, wrists, knee, etc.), replicating a subset of the setup used in the [OPPORTUNITY dataset](http://www.opportunity-project.eu/challengeDataset.html) for Human Activity Recognition:

&nbsp;

<img src="https://github.com/mosamadeeb/MultiSensorHardwareSetup/assets/52977072/f347b18a-2d14-4e27-84ee-f41492f3f8fe" height="400">

&nbsp;

### Setup

- Each group of closely placed sensors is connected on an I2C bus to an MCU that collects the data, filters it, serializes it, and sends it over Bluetooth LE to a separate on-body device.

- The on-body device [synchronizes the packets](https://github.com/mosamadeeb/MultiSensorScripts/blob/main/bleak_client.py) from the MCU nodes, and sends the aggregated data to a Python server.

- The Python server performs inference on the data using a [Random Forest ML model](https://github.com/Noor-Z1/Graduation-Project) for activity recognition (trained using this same setup).

- The results of the inference are displayed on an [Android app](https://github.com/barisuu/HealthActivityApp) connected to the same local WiFi network as the server.

---

## Features

- Compatibility with different MCU architectures using the [PlatformIO](https://platformio.org/) ecosystem (we're using Xtensa and ARM devices together).

- Stabilized sensor output using a hardware (built-in) low-pass filter and a software Kalman filter.

- Efficient Bluetooth LE operation that uses advertising for connecting and notifications for transmitting the data periodically.

- Compact serialized packets using [MessagePack](https://msgpack.org/) to fit the MTU size for Bluetooth LE.

- Reliable recovery implementation that detects any disconnections and reinitializes the sensors/connection.

## Hardware

- 9x [MPU-6050](https://invensense.tdk.com/products/motion-tracking/6-axis/mpu-6050/) 6-axis accelerometer + gyroscope
- 4x [QMC5883L](https://www.qstcorp.com/en_comp_prod/QMC5883L) 3-axis magnetometer
- 2x ESP32 MCU with **two** I2C controllers and Bluetooth LE
  - e.g. [ESP32-DevKitC V4](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/hw-reference/esp32/get-started-devkitc.html)
- 2x nRF52840 MCU with **one** I2C controller and Bluetooth LE
  - e.g. [Seeed Studio XIAO nRF52840](https://wiki.seeedstudio.com/XIAO_BLE/)
- Linux device with Bluetooth LE (bluez) to run the Python server
  - e.g. [Raspberry Pi 5](https://www.raspberrypi.com/products/raspberry-pi-5/)

# bno_055_arduino_ros2_serial
Package Containing ROS2 Package and Arduino script to establish serial link between them

## Installation

Upload the [Arduino code](https://github.com/ASU-RISE-Lab/bno_055_arduino_ros2_serial/blob/main/arduino_code/multi_imu/multi_imu.ino) to the Arduino. 

The Arduino code is based on the [Adafruit BNO055 library](https://www.arduino.cc/reference/en/libraries/adafruit-bno055/)

## Usage

In order to run the node. You need to run the [serial_link.py](https://github.com/ASU-RISE-Lab/bno_055_arduino_ros2_serial/blob/main/src/serial_link.py). 

Please run the following command in the terminal:

```bash
python3 bno_055_arduino_ros2_serial/src/serial_link.py
```

The node will publish the IMU data to the topic: /IMU_Data. 

The custom message type can be found in the [ImuData.msg](https://github.com/ASU-RISE-Lab/bno_055_arduino_ros2_serial/blob/main/msg/ImuData.msg) file.

Even though the Node is written for 4 IMUs, the Arduino code is written for 2 IMUs. So the Arduino code needs to be modified to work with 4 IMUs. 

<!-- Need to add wiring diagram -->






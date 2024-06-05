# Beach Cleaner Robot - Autonomous Mobile Robot Development

This repository contains code and instructions for the development of an autonomous mobile beach cleaner robot. The robot is designed to clean beaches and is controlled using a Raspberry Pi 3. The software stack is built on the ROS Kinetic distribution.

## STATUS:
the rosserial is in conflict when you set up all the sensors. I don't know yet where and when I made that conflict. I think it is the uart configuration. 

 
## Arduino Code for Arduino:

- `encoder_test`: Use this code to determine the counts your encoder generates per revolution of the motor.
- `pid_test_speed_mechanism`: This code provides a discrete PID control of a motor using the LM298 driver.
- `pid_test_speed2`: Here, you'll find code for discrete PID control of a motor with an MDD10A driver.
- `ros_speed2`: This code implements ROS communication (rosserial - twist message) and discrete PID control of a motor with an MDD10A driver.
- `kinematics`: This simulation demonstrates the kinematics of a mobile robot with a 4-wheel configuration.

## rosserial_python for Multiple Arduinos (Update)

To run multiple Arduinos, you need to make a modification to the `serial_node.py` file. More information can be found [here](https://answers.ros.org/question/12684/using-multiple-arduinos-running-multiple-nodes/). (CHECK serial_node.py)

```python
rospy.init_node("serial_node", anonymous=True)
rospy.loginfo("ROS Serial Python Node {}".format(rospy.get_caller_id()))
```

On the Raspberry Pi, locate the following file:

```bash
/opt/ros/{DISTRO}/lib/rosserial_python/serial_node.py
```
on computer you should locate your rosserial package.

To run with a single Arduino Nano, use this command:

```bash
rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0
```

To run with multiple Arduinos, use:

```bash
roslaunch launch_rosserial rosserial.launch
```

## IMU BNO055

Before proceeding, ensure that you have set up the Raspberry Pi's I2C clock as described [here](https://learn.adafruit.com/circuitpython-on-raspberrypi-linux/i2c-clock-stretching). For using the BNO055 IMU sensor with ROS, follow this tutorial: [How to Publish IMU Data Using ROS and the BNO055 IMU Sensor](https://automaticaddison.com/how-to-publish-imu-data-using-ros-and-the-bno055-imu-sensor/).

## GPS UBLOX

For using a UBLOX GPS receiver with ROS Kinetic, follow this tutorial: [nmea_navsat_driver](http://wiki.ros.org/nmea_navsat_driver).

Here's a general outline of the process:

1. Clone the repository for the Kinetic branch (I'm using kinetic):


```plaintext
git clone -b kinetic-devel ...
```

3. Copy the folders "launch," "scripts," and "src" from the master branch and paste them into the kinetic branch.

4. Copy all the files from the "scripts" folder and paste them into the "src" folder. Afterward, you can delete the now-empty "script" folder.

5. Update the first line in all the copied files to:

``` python
#! /usr/bin/env python
```

Don't forget to update your launch file accordingly.

If you plan to use UART pins on your Raspberry Pi, follow this guide for configuration: [Raspberry Pi UART Communication Using Python and C](https://www.electronicwings.com/raspberry-pi/raspberry-pi-uart-communication-using-python-and-c).

### USE ublox_files and copy all the files in nmea_navsat_driver and replace

## USB CAM

To set up a USB camera, you can follow the tutorial available here: [usb_cam]( http://wiki.ros.org/usb_cam).

Here are the basic steps:
1. Clone the USB cam repository

``` bash
git clone https://github.com/ros-drivers/usb_cam.git
```
2. Build package
```bash
catkin_make
```
3. Launch and visualize
```
roslaunch usb_cam usb_cam.launch

rosrun image_view image_view image:=/usb_cam/image_raw
```

## BEACH CLEANER ROBOT

To run the beach cleaner robot with all the included sensors, use the following command on your Raspberry Pi or Jetson:

```
roslaunch launch_rosserial beach_robot_bringup.launch
```
To control the robot using a twist message publisher on your computer, run:

```
roslaunch launchrosserial teleop.launch
```

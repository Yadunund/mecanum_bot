# mecanum_bot

![](docs/media/ramu.jpg)

## RAMU (Really Awesome Mobile Unit)

RAMU is a low-cost, wheel encoder-less, holonomic mobile base that is capable of autonomous navigation. RAMU is powered by ROS 2 and Navigation 2 systems. This repository contains some information on RAMU along with ROS 2 packages and launch files to get RAMU up and running.

#### Click the image below to watch RAMU autonomously navigate in an indoor environment. 

[![](docs/media/autonomous_navigation.jpeg)](https://www.youtube.com/watch?v=9J7-lWX2Y_A&feature=youtu.be)


## Hardware Design

![](docs/media/ramu_cad.jpg)

RAMU's chassis is constructed from 20x20 aluminium extrusion members which are connected together with 3D printed joints. The motor, battery, electronics and sensors mounts are also 3D printed. The CAD is open sourced via Onshape and can be accessed [here](https://cad.onshape.com/documents/ce8bdc9f696707bc50d70932/w/ef0d68341801b7ffe4e012c3/e/bb0d1657fc848bdd1addb320).

Documentation of the build process is found [here](https://yadunundvijay.com)

### BOM
1. [Raspberry Pi 3B](https://www.raspberrypi.org/products/raspberry-pi-3-model-b/)
2. [Arduino Uno](https://store.arduino.cc/usa/arduino-uno-rev3)
3. [DC geared motors x4](https://www.aliexpress.com/item/32999954820.html?spm=a2g0s.9042311.0.0.65cf4c4dNaN8tW)
4. [Motor drivers x2](https://www.aliexpress.com/item/4000099388630.html?spm=a2g0s.9042311.0.0.65cf4c4dNaN8tW)
5. [Mecanum wheels 96mm diameter x4](https://www.aliexpress.com/item/4001118452729.html?spm=a2g0s.9042311.0.0.65cf4c4dNaN8tW)
6. [2D lidar](https://www.aliexpress.com/item/4000018415971.html?spm=a2g0s.9042311.0.0.65cf4c4dNaN8tW)

## Software Setup

## Install 

Install dependencies
```bash
python3 -m pip install pyserial
```
# orion_web_interface
[![language](https://img.shields.io/badge/language-react-239120)](#)
[![framework](https://img.shields.io/badge/framework-astro-239120)](#)
[![OS](https://img.shields.io/badge/OS-Ubuntu_24.04-0078D4)](#)
[![CPU](https://img.shields.io/badge/CPU-x86%2C%20x64%2C%20ARM%2C%20ARM64-FF8C00)](#)
[![GitHub release](https://img.shields.io/badge/release-v2.0.15-4493f8)](#)
[![GitHub release date](https://img.shields.io/badge/release_date-february_2025-96981c)](#)
[![GitHub last commit](https://img.shields.io/badge/last_commit-april_2025-96981c)](#)

⭐ Star us on GitHub — it motivates us a lot!

## Table of Contents
- [About](#-about)
- [Demostration](#-demostration)
- [How to Build](#-how-to-build)
- [License](#-license)

## 🚀 About

**orion web interface** is a package for ROS2 Jazzy that allow us to deploy a web app, and a web server, in which we can publish and suscribe to a string topic, and present in the webpage the depth camera and normal camera captured video.

## 🎥 Demostration


https://github.com/user-attachments/assets/82df0b2a-f326-4097-bd01-7f44de2f1de3



## 📝 How to Build

To build the packages, follow these steps:

```shell
# First you need to clone the repository in your workspace
cd ~/ros2_ws/src
git clone https://github.com/Tesis-ORION/orion_web_interface

# Now you need to install the dependencies
sudo apt install ros-jazzy-rosbridge-server nodejs

# Next you need to compile the package
cd ~/ros2_ws
colcon build --packages-select orion_web_interface

# Now install node package dependencies and run frontend
cd ros2_ws/src/orion_web_interface/orion_web_interface/astro-orion/
npm install
npm run dev
# It will be deployed in http://localhost:4321/

# Now run your depth camera package (If you aren't using ydlidar, change the topic name in frontend folder, Home.jsx file)
ros2 launch depth_ydlidar_os30a apc_camera_launch.py

# At last run backend with rosbridge server
ros2 launch orion_web_interface backend.launch.py
```

## 📃 License

orion_web_interface is available under the BSD-3-Clause license. See the LICENSE file for more details.

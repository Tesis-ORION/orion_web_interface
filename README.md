# orion_web_interface
[![language](https://img.shields.io/badge/language-python-239120)](#)
[![OS](https://img.shields.io/badge/OS-Ubuntu_24.04-0078D4)](#)
[![CPU](https://img.shields.io/badge/CPU-x86%2C%20x64%2C%20ARM%2C%20ARM64-FF8C00)](#)
[![GitHub release](https://img.shields.io/badge/release-v1.0.0-4493f8)](#)
[![GitHub release date](https://img.shields.io/badge/release_date-february_2025-96981c)](#)
[![GitHub last commit](https://img.shields.io/badge/last_commit-february_2025-96981c)](#)

⭐ Star us on GitHub — it motivates us a lot!

## Table of Contents
- [About](#-about)
- [Demostration](#-demostration)
- [How to Build](#-how-to-build)
- [License](#-license)

## 🚀 About

**orion web interface** is a package for ROS2 Jazzy that allow us to deploy a web app, and a web server, in which we can publish and suscribe to a string topic, and present in the webpage the depth camera captured video.

## 🎥 Demostration
https://github.com/user-attachments/assets/4d03d90e-e2b5-4ba4-900e-c8d82f192baf

## 📝 How to Build

To build the packages, follow these steps:

```shell
# First you need to clone the repository in your workspace
cd ~/ros2_ws/src
git clone https://github.com/Tesis-ORION/orion_web_interface

# Now you need to install the dependencies
sudo apt install ros-jazzy-rosbridge-server

# Next you need to compile the package and launch the project
cd ~/ros2_ws
colcon build --packages-select orion_web_interface
ros2 launch orion_web_interface backend.launch.py

# At last you need to double click index.html locatd in orion_web_interface/orion_web_interface/index.html
```

## 📃 License

orion_web_interface is available under the BSD-3-Clause license. See the LICENSE file for more details.

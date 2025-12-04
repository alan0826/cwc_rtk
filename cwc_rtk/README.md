# NTRIP Client Script

## Introduction
* The script is a ROS-based NTRIP (Networked Transport of RTCM via Internet Protocol) client designed to receive RTCM (Radio Technical Commission for Maritime Services) data from an NTRIP server and publish it to a ROS topic. 


## How to install 
### Install
* cd ~/catkin_ws && catkin_make --only-pkg-with-deps cwc_rtk
### Ensure permissions after installation.
* sudo chmod +x  /your_path/combined.py

---
## How to Use

* sudo chmod 777   /dev/your_serial_port
* roslaunch cwc_rtk receive_correction_and_send_to_serial.launch

---

## Additional Notes
* Be sure to make code changes according to different devices in combined.py. 
* If using u-blox, only publishing the Topic is required, and writing to the port is unnecessary. 
* If using NovAtel, writing to the port is necessary.


# **Zenai Roboto Car**
---
**A self-driving robot car built from scratch**

[//]: # (Image References)
[image0]: ./catkin_ws/images/zenai.png "Zenai Robot Car"
[image1]: ./catkin_ws/images/zenai_making.png "Zenai HW"
[image2]: ./catkin_ws/images/track.jpg "Camera Image"
[image3]: ./catkin_ws/images/track_undist.jpg "Camera Image Undistorted"

![][image0]
## Project
---
This is a self-initiated personal project and **is a WORK-IN-PROGRESS!**

My goal in this project is to build a robot car from scratch. It currently uses the below off-the-shelf hardware, with Ubuntu 16.04, ROS, and C++.

- Raspberry Pi 3B+ with Boardcom BCM 2837 64-bit ARMv7 Quad Core 1200MHz processor and 1GB SDRAM@400MHz
- Raspberry Pi 4 Camera Module 5 Megapixels 1080p OV5647 Sensor Adjustable Focus Wide Angle Fish-Eye Camera Lenses
- 4WD Robot Chassis Kit with 4 TT Motor for Raspberry Pi
- Super Small PCB Board H-Bridge L9110 2 Way Motor Driver Module
- Miuzei Raspberry Pi 3B+ Battery Pack Expansion Board and UPS 3800mAH Battery Pack

All the hardware is bought online on Amazon.com.

## Hardware Setup
---
![][image1]


## Sample Camera Calibration Result
---

Camera Image | Camera Image Undistorted
:---------:|:----------:
![][image2]|![][image3]

# Directory Structure
The directory structure of this repository is as follows:

```
README.md
|
catkin_ws
|
|___launch
|   |   
|   |   zenai.launch
|   
|___images
|   |   
|   |   <image_files>
|   
|___src
    |
	|___<module_name>
    |   |   CMakeList.txt
    |   |   package.xml
    |   |
    |   |___include
    |   |   
    |   |___launch
    |   |
    |   |___src
    |
	.
	.
	.
	|___<module_name>
    |   |   CMakeList.txt
    |   |   package.xml
    |   |
    |   |___include
    |   |   
    |   |___launch
    |   |
    |   |___src
```

## Code Style
---
My goal is to stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

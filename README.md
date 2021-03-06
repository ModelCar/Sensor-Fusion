<!-- MarkdownTOC -->

- [Overview](#overview)
  - [Introduction](#introduction)
  - [Framework](#framework)
  - [Future Work](#future-work)
- [Run Projects](#run-projects)
  - [Install KFilter Library](#KFilter)
  - [Install OpenCV Library](#OpenCV)
  - [Compile](#compile)

<!-- /MarkdownTOC -->

<a name="overview"></a>
#Overview

<a name="introduction"></a>
##Introduction
I introduced sensor fusion capabilities to the iron car, which could fuse the measurements from ultrasonic sensors and stereo camera to give a good estimate of the states of moving object in front of the iron car. The Extended Kalman Filter is used for the sensor fusion. In the future, measurements from different other types of sensors could be incorporated to the framework. With the state information at hand, it becomes possible in the future for the iron car to have adaptive cruise control and overtaking functionalities. Please refer to the final presentation for detailed information of the project. 

![Intro Picture](https://lh3.googleusercontent.com/WzXFOyrc3XWxJtpylGiaOcPw6tCq7MDHeBbfhJ064v8PZ4i3zUf_5GPAIjDkfJMWEfmv9by3=s600 "Selection_001.png")

<a name="framework"></a>
##Framework
![Framework Picture](https://lh3.googleusercontent.com/U4w8gCDrOTOWNBfiM7cKV6zIS45p175P8f33vgsn9h3FRrK2cQRzBiIcvgnpkK-wTfzVcD7g=s500 "Selection_002.png")

<a name="future-work"></a>
##Future Work

- Set up tri-aural ultrasonic sensor in the iron car.
- Design an algorithm to extract distance information from the depth map generated by stereo camera.
- Implement CV algorithms to do obstacle classification and add this information to the sensor fusion framework, which let the car better decide its action, be it overtaking or following the object.

<a name="run_projects"></a>
#Run Projects

<a name="KFilter"></a>
##Install KFilter Library
A small static library (libkalman.a) will be installed in /usr/local/lib. A directory named 'kalman' containing all necessary include files will be installed in /usr/local/include.

Run the following commands in Linux under the Sensor_Fusion folder:
```{r, engine='sh', count_lines}
make
su
install
```

<a name="OpenCV"></a>
##Install OpenCV Library
This OpenCV library is needed only if you would like to run the project in videoSimulation folder. 

Run the following commands in Linux:
```{r, engine='sh', count_lines}
sudo apt-get update && sudo apt-get upgrade && sudo rpi-update
sudo apt-get install build-essential
sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
sudo apt-get install python-dev python-numpy libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev
git clone https://github.com/Itseez/opencv.git
git clone https://github.com/Itseez/opencv_contrib.git
cd opencv
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local \
 -D INSTALL_PYTHON_EXAMPLES=ON \
 -D INSTALL_C_EXAMPLES=OFF \
 -D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib/modules \
 -D BUILD_EXAMPLES=ON ..
make -j $(nproc)
sudo /bin/bash -c 'echo "/usr/local/lib" > /etc/ld.so.conf.d/opencv.conf'
sudo make install && sudo ldconfig
sudo apt-get install libboost-system-dev libboost-thread-dev libboost-timer-dev libboost-chrono-dev libboost-filesystem-dev
```

<a name="Compile"></a>
##Compile
run **make** under the fusionSimulation and/or viodeSimulation folder to create the executable. 

Note: You could run simulation.m in Matlab to create simulated sensor measurements data used for sensor fusion. 

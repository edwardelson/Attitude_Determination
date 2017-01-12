#ABSTRACT
This is a model CubeSat used to perform overall testing of the subsystems that we have previously built. The subsystems are:   
   
(1) Reaction wheel motor. This is often used in a satellite system for attitude control such as pointing to a certain direction i.e. earth (which is useful for taking image of earth).   
   
(2) Inertial Measurement Unit + Kalman Filter. IMU consists of accelerometer, gyroscope and magnetometer. This is used to obtain orientation of a satellite body. However, the data from these sensors are very noisy, hence we implement Kalman Filter to minimize the error.    
   
(3) Sun sensor. This is used to obtain the location of sun with respect to the satellite body. The attitude of the satellite can then be estimated with the sun sensor.     

![Singapore Space Challenge](https://googledrive.com/host/0B3qXE5D5r-qCYzhBUF81cFN6Wlk/12244448_1076856415681904_6874440074577415756_o.jpg)  
_Our CubeSat model in Singapore Space Challenge 2015 - First Runner-Up!_   
    
![Model CubeSat](https://googledrive.com/host/0B3qXE5D5r-qCYzhBUF81cFN6Wlk/IMG_20151102_203244_HDR.jpg)  
  
The device is powered by an ARM Cortex-M4F STM32 NUCLEO based driver board. Data is transmitted wirelessly to the main computer (to simulate ground station) and plotted in real time.
  
![Sun Sensor Testing](https://googledrive.com/host/0B3qXE5D5r-qCYzhBUF81cFN6Wlk/IMG_20151019_221934_HDR.jpg)
_Sun Sensor under Testing_  

This project is part of EG3301 [Design Centric Programme](http://www.eng.nus.edu.sg/edic/about.html) (Satellite System Design Track) module at the National University of Singapore.
    
   
>[Back to List of Projects](https://edwardelson.github.io)  
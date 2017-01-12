#ABSTRACT
This is a model CubeSat used to perform overall testing of the subsystems that we have previously built. The subsystems are:   
   
(1) Reaction wheel motor. This is often used in a satellite system for attitude control such as pointing to a certain direction i.e. earth (which is useful for taking image of earth).   
   
(2) Inertial Measurement Unit + Kalman Filter. IMU consists of accelerometer, gyroscope and magnetometer. This is used to obtain orientation of a satellite body. However, the data from these sensors are very noisy, hence we implement Kalman Filter to minimize the error.    
   
(3) Sun sensor. This is used to obtain the location of sun with respect to the satellite body. The attitude of the satellite can then be estimated with the sun sensor.     

![Singapore Space Challenge](https://dl.dropboxusercontent.com/s/y3ok9bjl0iokjir/space_challenge.jpg?dl=0)  
_Our CubeSat model in Singapore Space Challenge 2015 - First Runner-Up!_   
    
![Model CubeSat](https://dl.dropboxusercontent.com/s/vtn1witlhqcmxv3/adcs.jpg?dl=0)  
  
The device is powered by an ARM Cortex-M4F STM32 NUCLEO based driver board. Data is transmitted wirelessly to the main computer (to simulate ground station) and plotted in real time.
  
![Sun Sensor Testing](https://dl.dropboxusercontent.com/s/m5ftxcnadlga46e/sun_sensor.jpg?dl=0)
_Sun Sensor under Testing_  

This project is part of EG3301 [Design Centric Programme](http://www.eng.nus.edu.sg/edic/about.html) (Satellite System Design Track) module at the National University of Singapore.
    
   
>[Back to List of Projects](https://edwardelson.github.io)  

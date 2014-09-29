Multi-Platform-Vision-System
============================

Code repository for paper id: 

Data
____
Image stream files can be found at: [http://multiplatformvisionsystem.sourceforge.net](http://multiplatformvisionsystem.sourceforge.net).

Platforms
_________

There are three separate versions of system. Two are targetted at the Darwin-OP platform ([www.robotis.com/xe/darwin_en](www.robotis.com/xe/darwin_en)) and the third at the Raspberry-Pi platform ([www.raspberrypi.org/](www.raspberrypi.org/)). 

Compiling and Running
_____________________

The two version for running on the Darwin-OP platform are both based off the NUbots software platform and can be built using the make utility on debian based systems (see https://github.com/nubots/robocup for further instructions).

The version for running on the Raspberry-Pi requires the open-source [wiringpi](http://wiringpi.com/) library and must be compiled on the board itself using the makefile provided in *RPi/build/* resulting a binary named *Vision* in the same folder which can then be run. 

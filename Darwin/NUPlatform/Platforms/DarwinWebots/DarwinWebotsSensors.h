/*! @file DarwinWebotsSensors.h
    @brief Declaration of Darwin in Webots sensors class

    @author Jason Kulk
 
    @class DarwinWebotsSensors
    @brief A Darwin in Webots sensors
 
    @author Jason Kulk
 
  Copyright (c) 2009 Jason Kulk
 
    This file is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This file is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with NUbot.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef DARWINWEBOTSSENSORS_H
#define DARWINWEBOTSSENSORS_H

#include "NUPlatform/NUSensors.h"
#include "Infrastructure/NUData.h"
#include "DarwinWebotsPlatform.h"
#include "DarwinJointMapping.h"

#include <vector>
#include <string>
#include <fstream>

#include "webots/Robot.hpp"
#include "NUPlatform/Platforms/Webots/JRobot.h"


class DarwinWebotsSensors : public NUSensors
{
public:
    DarwinWebotsSensors(DarwinWebotsPlatform* platform);
    ~DarwinWebotsSensors();
    
private:
    void getSensorsFromWebots();
    void enableSensorsInWebots();
    
    void copyFromHardwareCommunications();
    void copyFromJoints();
    void copyFromAccelerometerAndGyro();
    void copyFromDistance();
    void copyFromFootSole();
    void copyFromFootBumper();
    void copyFromGPS();
    void copyFromCompass();
    
private:
    const int m_simulation_step;                                //!< the refresh period of the sensor data in milliseconds. Robotstadium's timestep is fixed at 40ms
    
    DarwinWebotsPlatform* m_platform;                              //!< a pointer to the platform, in particular in webots this inherits from webots::Robot so use it to access devices
    // Sensors
    static std::vector<std::string> m_servo_names;                        //!< a vector of the names of each servo in the Webot Darwin
    std::vector<webots::Servo*> m_servos;                           //!< a vector containing pointers to each of the servos in the Webot Darwin.
    webots::Accelerometer* m_accelerometer;                     //!< a pointer to the robot's accelerometer
    webots::Gyro* m_gyro;                                       //!< a pointer to the robot's gyrometer
    static vector<std::string> m_distance_names;                     //!< a vector of the names of each of the distance sensors in Webot Darwin
    std::vector<webots::DistanceSensor*> m_distance_sensors;         //!< a vector containing pointers to each of the distance sensors in the Webot Darwin
    static vector<std::string> m_foot_sole_names;                    //!< a vector of the names of each of the foot touch sensors in Webot Darwin
    std::vector<webots::TouchSensor*> m_foot_sole_sensors;           //!< a vector of pointers to each of the foot force sensors in the Webot Darwin
    static vector<std::string> m_foot_bumper_names;                  //!< a vector of the foot bumper names
    std::vector<webots::TouchSensor*> m_foot_bumper_sensors;         //!< a vector of pointers to buttons
    webots::GPS* m_gps;                                         //!< a pointer to the gps module of the robot available for testing!
    webots::Compass* m_compass;                                 //!< a pointer to the compass module of the robot available for testing (only in PRO)
    
    std::vector<NUData::id_t*> m_joint_ids;
    std::vector<float> m_previous_positions;
    std::vector<float> m_previous_velocities;
};

#endif


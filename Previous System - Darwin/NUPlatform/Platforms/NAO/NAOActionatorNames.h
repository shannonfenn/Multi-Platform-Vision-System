/*! @file NAOActionatorNames.h
    @brief Definition of NAO actionator names (ie. Device names to be used with dcm)
 
    @author Jason Kulk
 
 Copyright (c) 2009, 2010 Jason Kulk
 
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

#ifndef NAOACTIONATORNAMES_H
#define NAOACTIONATORNAMES_H

#include <string>
using namespace std;

#define DN_PREFIX           string("")
#define DN_POSTFIX          string("/Actuator/Value")

// Joint Names
#define HEAD_YAW             DN_PREFIX + string("HeadYaw") + DN_POSTFIX
#define HEAD_PITCH           DN_PREFIX + string("HeadPitch") + DN_POSTFIX
#define L_SHOULDER_ROLL      DN_PREFIX + string("LShoulderRoll") + DN_POSTFIX
#define L_SHOULDER_PITCH     DN_PREFIX + string("LShoulderPitch") + DN_POSTFIX
#define L_ELBOW_YAW          DN_PREFIX + string("LElbowYaw") + DN_POSTFIX 
#define L_ELBOW_ROLL         DN_PREFIX + string("LElbowRoll") + DN_POSTFIX
#define R_SHOULDER_ROLL      DN_PREFIX + string("RShoulderRoll") + DN_POSTFIX
#define R_SHOULDER_PITCH     DN_PREFIX + string("RShoulderPitch") + DN_POSTFIX
#define R_ELBOW_YAW          DN_PREFIX + string("RElbowYaw") + DN_POSTFIX 
#define R_ELBOW_ROLL         DN_PREFIX + string("RElbowRoll") + DN_POSTFIX
#define L_HIP_YAWPITCH       DN_PREFIX + string("LHipYawPitch") + DN_POSTFIX 
#define L_HIP_ROLL           DN_PREFIX + string("LHipRoll") + DN_POSTFIX 
#define L_HIP_PITCH          DN_PREFIX + string("LHipPitch") + DN_POSTFIX
#define L_KNEE_PITCH         DN_PREFIX + string("LKneePitch") + DN_POSTFIX
#define L_ANKLE_PITCH        DN_PREFIX + string("LAnklePitch") + DN_POSTFIX 
#define L_ANKLE_ROLL         DN_PREFIX + string("LAnkleRoll") + DN_POSTFIX
#define R_HIP_YAWPITCH       DN_PREFIX + string("RHipYawPitch") + DN_POSTFIX
#define R_HIP_ROLL           DN_PREFIX + string("RHipRoll") + DN_POSTFIX 
#define R_HIP_PITCH          DN_PREFIX + string("RHipPitch") + DN_POSTFIX
#define R_KNEE_PITCH         DN_PREFIX + string("RKneePitch") + DN_POSTFIX
#define R_ANKLE_PITCH        DN_PREFIX + string("RAnklePitch") + DN_POSTFIX 
#define R_ANKLE_ROLL         DN_PREFIX + string("RAnkleRoll") + DN_POSTFIX

// Device names for position actuators
#define HEAD_YAW_POSITION            DN_PREFIX + string("HeadYaw/Position") + DN_POSTFIX
#define HEAD_PITCH_POSITION          DN_PREFIX + string("HeadPitch/Position") + DN_POSTFIX
#define L_SHOULDER_ROLL_POSITION     DN_PREFIX + string("LShoulderRoll/Position") + DN_POSTFIX
#define L_SHOULDER_PITCH_POSITION    DN_PREFIX + string("LShoulderPitch/Position") + DN_POSTFIX
#define L_ELBOW_YAW_POSITION         DN_PREFIX + string("LElbowYaw/Position") + DN_POSTFIX 
#define L_ELBOW_ROLL_POSITION        DN_PREFIX + string("LElbowRoll/Position") + DN_POSTFIX
#define R_SHOULDER_ROLL_POSITION     DN_PREFIX + string("RShoulderRoll/Position") + DN_POSTFIX
#define R_SHOULDER_PITCH_POSITION    DN_PREFIX + string("RShoulderPitch/Position") + DN_POSTFIX
#define R_ELBOW_YAW_POSITION         DN_PREFIX + string("RElbowYaw/Position") + DN_POSTFIX 
#define R_ELBOW_ROLL_POSITION        DN_PREFIX + string("RElbowRoll/Position") + DN_POSTFIX
#define L_HIP_YAWPITCH_POSITION      DN_PREFIX + string("LHipYawPitch/Position") + DN_POSTFIX 
#define L_HIP_ROLL_POSITION          DN_PREFIX + string("LHipRoll/Position") + DN_POSTFIX 
#define L_HIP_PITCH_POSITION         DN_PREFIX + string("LHipPitch/Position") + DN_POSTFIX
#define L_KNEE_PITCH_POSITION        DN_PREFIX + string("LKneePitch/Position") + DN_POSTFIX
#define L_ANKLE_PITCH_POSITION       DN_PREFIX + string("LAnklePitch/Position") + DN_POSTFIX 
#define L_ANKLE_ROLL_POSITION        DN_PREFIX + string("LAnkleRoll/Position") + DN_POSTFIX
#define R_HIP_YAWPITCH_POSITION      DN_PREFIX + string("RHipYawPitch/Position") + DN_POSTFIX
#define R_HIP_ROLL_POSITION          DN_PREFIX + string("RHipRoll/Position") + DN_POSTFIX 
#define R_HIP_PITCH_POSITION         DN_PREFIX + string("RHipPitch/Position") + DN_POSTFIX
#define R_KNEE_PITCH_POSITION        DN_PREFIX + string("RKneePitch/Position") + DN_POSTFIX
#define R_ANKLE_PITCH_POSITION       DN_PREFIX + string("RAnklePitch/Position") + DN_POSTFIX 
#define R_ANKLE_ROLL_POSITION        DN_PREFIX + string("RAnkleRoll/Position") + DN_POSTFIX

// Device names for hardness actuators
#define HEAD_YAW_HARDNESS            DN_PREFIX + string("HeadYaw/Hardness") + DN_POSTFIX
#define HEAD_PITCH_HARDNESS          DN_PREFIX + string("HeadPitch/Hardness") + DN_POSTFIX
#define L_SHOULDER_ROLL_HARDNESS     DN_PREFIX + string("LShoulderRoll/Hardness") + DN_POSTFIX
#define L_SHOULDER_PITCH_HARDNESS    DN_PREFIX + string("LShoulderPitch/Hardness") + DN_POSTFIX
#define L_ELBOW_YAW_HARDNESS         DN_PREFIX + string("LElbowYaw/Hardness") + DN_POSTFIX 
#define L_ELBOW_ROLL_HARDNESS        DN_PREFIX + string("LElbowRoll/Hardness") + DN_POSTFIX
#define R_SHOULDER_ROLL_HARDNESS     DN_PREFIX + string("RShoulderRoll/Hardness") + DN_POSTFIX
#define R_SHOULDER_PITCH_HARDNESS    DN_PREFIX + string("RShoulderPitch/Hardness") + DN_POSTFIX
#define R_ELBOW_YAW_HARDNESS         DN_PREFIX + string("RElbowYaw/Hardness") + DN_POSTFIX 
#define R_ELBOW_ROLL_HARDNESS        DN_PREFIX + string("RElbowRoll/Hardness") + DN_POSTFIX
#define L_HIP_YAWPITCH_HARDNESS      DN_PREFIX + string("LHipYawPitch/Hardness") + DN_POSTFIX 
#define L_HIP_ROLL_HARDNESS          DN_PREFIX + string("LHipRoll/Hardness") + DN_POSTFIX 
#define L_HIP_PITCH_HARDNESS         DN_PREFIX + string("LHipPitch/Hardness") + DN_POSTFIX
#define L_KNEE_PITCH_HARDNESS        DN_PREFIX + string("LKneePitch/Hardness") + DN_POSTFIX
#define L_ANKLE_PITCH_HARDNESS       DN_PREFIX + string("LAnklePitch/Hardness") + DN_POSTFIX 
#define L_ANKLE_ROLL_HARDNESS        DN_PREFIX + string("LAnkleRoll/Hardness") + DN_POSTFIX
#define R_HIP_YAWPITCH_HARDNESS      DN_PREFIX + string("RHipYawPitch/Hardness") + DN_POSTFIX
#define R_HIP_ROLL_HARDNESS          DN_PREFIX + string("RHipRoll/Hardness") + DN_POSTFIX 
#define R_HIP_PITCH_HARDNESS         DN_PREFIX + string("RHipPitch/Hardness") + DN_POSTFIX
#define R_KNEE_PITCH_HARDNESS        DN_PREFIX + string("RKneePitch/Hardness") + DN_POSTFIX
#define R_ANKLE_PITCH_HARDNESS       DN_PREFIX + string("RAnklePitch/Hardness") + DN_POSTFIX 
#define R_ANKLE_ROLL_HARDNESS        DN_PREFIX + string("RAnkleRoll/Hardness") + DN_POSTFIX

// Device names for ear LEDs
#define LED_EAR_LEFT_0DEG            DN_PREFIX + string("Ears/Led/Left/0Deg") + DN_POSTFIX
#define LED_EAR_LEFT_36DEG           DN_PREFIX + string("Ears/Led/Left/36Deg") + DN_POSTFIX
#define LED_EAR_LEFT_72DEG           DN_PREFIX + string("Ears/Led/Left/72Deg") + DN_POSTFIX
#define LED_EAR_LEFT_108DEG          DN_PREFIX + string("Ears/Led/Left/108Deg") + DN_POSTFIX
#define LED_EAR_LEFT_144DEG          DN_PREFIX + string("Ears/Led/Left/144Deg") + DN_POSTFIX
#define LED_EAR_LEFT_180DEG          DN_PREFIX + string("Ears/Led/Left/180Deg") + DN_POSTFIX
#define LED_EAR_LEFT_216DEG          DN_PREFIX + string("Ears/Led/Left/216Deg") + DN_POSTFIX
#define LED_EAR_LEFT_252DEG          DN_PREFIX + string("Ears/Led/Left/252Deg") + DN_POSTFIX
#define LED_EAR_LEFT_288DEG          DN_PREFIX + string("Ears/Led/Left/288Deg") + DN_POSTFIX
#define LED_EAR_LEFT_324DEG          DN_PREFIX + string("Ears/Led/Left/324Deg") + DN_POSTFIX

#define LED_EAR_RIGHT_0DEG           DN_PREFIX + string("Ears/Led/Right/0Deg") + DN_POSTFIX
#define LED_EAR_RIGHT_36DEG          DN_PREFIX + string("Ears/Led/Right/36Deg") + DN_POSTFIX
#define LED_EAR_RIGHT_72DEG          DN_PREFIX + string("Ears/Led/Right/72Deg") + DN_POSTFIX
#define LED_EAR_RIGHT_108DEG         DN_PREFIX + string("Ears/Led/Right/108Deg") + DN_POSTFIX
#define LED_EAR_RIGHT_144DEG         DN_PREFIX + string("Ears/Led/Right/144Deg") + DN_POSTFIX
#define LED_EAR_RIGHT_180DEG         DN_PREFIX + string("Ears/Led/Right/180Deg") + DN_POSTFIX
#define LED_EAR_RIGHT_216DEG         DN_PREFIX + string("Ears/Led/Right/216Deg") + DN_POSTFIX
#define LED_EAR_RIGHT_252DEG         DN_PREFIX + string("Ears/Led/Right/252Deg") + DN_POSTFIX
#define LED_EAR_RIGHT_288DEG         DN_PREFIX + string("Ears/Led/Right/288Deg") + DN_POSTFIX
#define LED_EAR_RIGHT_324DEG         DN_PREFIX + string("Ears/Led/Right/324Deg") + DN_POSTFIX

// Device names for eye LEDs
#define LED_EYE_LEFT_RED_0DEG        DN_PREFIX + string("Face/Led/Red/Left/0Deg") + DN_POSTFIX
#define LED_EYE_LEFT_GREEN_0DEG      DN_PREFIX + string("Face/Led/Green/Left/0Deg") + DN_POSTFIX
#define LED_EYE_LEFT_BLUE_0DEG       DN_PREFIX + string("Face/Led/Blue/Left/0Deg") + DN_POSTFIX
#define LED_EYE_LEFT_RED_45DEG       DN_PREFIX + string("Face/Led/Red/Left/45Deg") + DN_POSTFIX
#define LED_EYE_LEFT_GREEN_45DEG     DN_PREFIX + string("Face/Led/Green/Left/45Deg") + DN_POSTFIX
#define LED_EYE_LEFT_BLUE_45DEG      DN_PREFIX + string("Face/Led/Blue/Left/45Deg") + DN_POSTFIX
#define LED_EYE_LEFT_RED_90DEG       DN_PREFIX + string("Face/Led/Red/Left/90Deg") + DN_POSTFIX
#define LED_EYE_LEFT_GREEN_90DEG     DN_PREFIX + string("Face/Led/Green/Left/90Deg") + DN_POSTFIX
#define LED_EYE_LEFT_BLUE_90DEG      DN_PREFIX + string("Face/Led/Blue/Left/90Deg") + DN_POSTFIX
#define LED_EYE_LEFT_RED_135DEG      DN_PREFIX + string("Face/Led/Red/Left/135Deg") + DN_POSTFIX
#define LED_EYE_LEFT_GREEN_135DEG    DN_PREFIX + string("Face/Led/Green/Left/135Deg") + DN_POSTFIX
#define LED_EYE_LEFT_BLUE_135DEG     DN_PREFIX + string("Face/Led/Blue/Left/135Deg") + DN_POSTFIX
#define LED_EYE_LEFT_RED_180DEG      DN_PREFIX + string("Face/Led/Red/Left/180Deg") + DN_POSTFIX
#define LED_EYE_LEFT_GREEN_180DEG    DN_PREFIX + string("Face/Led/Green/Left/180Deg") + DN_POSTFIX
#define LED_EYE_LEFT_BLUE_180DEG     DN_PREFIX + string("Face/Led/Blue/Left/180Deg") + DN_POSTFIX
#define LED_EYE_LEFT_RED_225DEG      DN_PREFIX + string("Face/Led/Red/Left/225Deg") + DN_POSTFIX
#define LED_EYE_LEFT_GREEN_225DEG    DN_PREFIX + string("Face/Led/Green/Left/225Deg") + DN_POSTFIX
#define LED_EYE_LEFT_BLUE_225DEG     DN_PREFIX + string("Face/Led/Blue/Left/225Deg") + DN_POSTFIX
#define LED_EYE_LEFT_RED_270DEG      DN_PREFIX + string("Face/Led/Red/Left/270Deg") + DN_POSTFIX
#define LED_EYE_LEFT_GREEN_270DEG    DN_PREFIX + string("Face/Led/Green/Left/270Deg") + DN_POSTFIX
#define LED_EYE_LEFT_BLUE_270DEG     DN_PREFIX + string("Face/Led/Blue/Left/270Deg") + DN_POSTFIX
#define LED_EYE_LEFT_RED_315DEG      DN_PREFIX + string("Face/Led/Red/Left/315Deg") + DN_POSTFIX
#define LED_EYE_LEFT_GREEN_315DEG    DN_PREFIX + string("Face/Led/Green/Left/315Deg") + DN_POSTFIX
#define LED_EYE_LEFT_BLUE_315DEG     DN_PREFIX + string("Face/Led/Blue/Left/315Deg") + DN_POSTFIX

#define LED_EYE_RIGHT_RED_0DEG       DN_PREFIX + string("Face/Led/Red/Right/0Deg") + DN_POSTFIX
#define LED_EYE_RIGHT_GREEN_0DEG     DN_PREFIX + string("Face/Led/Green/Right/0Deg") + DN_POSTFIX
#define LED_EYE_RIGHT_BLUE_0DEG      DN_PREFIX + string("Face/Led/Blue/Right/0Deg") + DN_POSTFIX
#define LED_EYE_RIGHT_RED_45DEG      DN_PREFIX + string("Face/Led/Red/Right/45Deg") + DN_POSTFIX
#define LED_EYE_RIGHT_GREEN_45DEG    DN_PREFIX + string("Face/Led/Green/Right/45Deg") + DN_POSTFIX
#define LED_EYE_RIGHT_BLUE_45DEG     DN_PREFIX + string("Face/Led/Blue/Right/45Deg") + DN_POSTFIX
#define LED_EYE_RIGHT_RED_90DEG      DN_PREFIX + string("Face/Led/Red/Right/90Deg") + DN_POSTFIX
#define LED_EYE_RIGHT_GREEN_90DEG    DN_PREFIX + string("Face/Led/Green/Right/90Deg") + DN_POSTFIX
#define LED_EYE_RIGHT_BLUE_90DEG     DN_PREFIX + string("Face/Led/Blue/Right/90Deg") + DN_POSTFIX
#define LED_EYE_RIGHT_RED_135DEG     DN_PREFIX + string("Face/Led/Red/Right/135Deg") + DN_POSTFIX
#define LED_EYE_RIGHT_GREEN_135DEG   DN_PREFIX + string("Face/Led/Green/Right/135Deg") + DN_POSTFIX
#define LED_EYE_RIGHT_BLUE_135DEG    DN_PREFIX + string("Face/Led/Blue/Right/135Deg") + DN_POSTFIX
#define LED_EYE_RIGHT_RED_180DEG     DN_PREFIX + string("Face/Led/Red/Right/180Deg") + DN_POSTFIX
#define LED_EYE_RIGHT_GREEN_180DEG   DN_PREFIX + string("Face/Led/Green/Right/180Deg") + DN_POSTFIX
#define LED_EYE_RIGHT_BLUE_180DEG    DN_PREFIX + string("Face/Led/Blue/Right/180Deg") + DN_POSTFIX
#define LED_EYE_RIGHT_RED_225DEG     DN_PREFIX + string("Face/Led/Red/Right/225Deg") + DN_POSTFIX
#define LED_EYE_RIGHT_GREEN_225DEG   DN_PREFIX + string("Face/Led/Green/Right/225Deg") + DN_POSTFIX
#define LED_EYE_RIGHT_BLUE_225DEG    DN_PREFIX + string("Face/Led/Blue/Right/225Deg") + DN_POSTFIX
#define LED_EYE_RIGHT_RED_270DEG     DN_PREFIX + string("Face/Led/Red/Right/270Deg") + DN_POSTFIX
#define LED_EYE_RIGHT_GREEN_270DEG   DN_PREFIX + string("Face/Led/Green/Right/270Deg") + DN_POSTFIX
#define LED_EYE_RIGHT_BLUE_270DEG    DN_PREFIX + string("Face/Led/Blue/Right/270Deg") + DN_POSTFIX
#define LED_EYE_RIGHT_RED_315DEG     DN_PREFIX + string("Face/Led/Red/Right/315Deg") + DN_POSTFIX
#define LED_EYE_RIGHT_GREEN_315DEG   DN_PREFIX + string("Face/Led/Green/Right/315Deg") + DN_POSTFIX
#define LED_EYE_RIGHT_BLUE_315DEG    DN_PREFIX + string("Face/Led/Blue/Right/315Deg") + DN_POSTFIX

// Device names for the chest LEDs
#define LED_CHEST_RED                DN_PREFIX + string("ChestBoard/Led/Red") + DN_POSTFIX
#define LED_CHEST_GREEN              DN_PREFIX + string("ChestBoard/Led/Green") + DN_POSTFIX
#define LED_CHEST_BLUE               DN_PREFIX + string("ChestBoard/Led/Blue") + DN_POSTFIX

// Device names for the feet LEDs
#define LED_FOOT_LEFT_RED            DN_PREFIX + string("LFoot/Led/Red") + DN_POSTFIX
#define LED_FOOT_LEFT_GREEN          DN_PREFIX + string("LFoot/Led/Green") + DN_POSTFIX
#define LED_FOOT_LEFT_BLUE           DN_PREFIX + string("LFoot/Led/Blue") + DN_POSTFIX

#define LED_FOOT_RIGHT_RED           DN_PREFIX + string("RFoot/Led/Red") + DN_POSTFIX
#define LED_FOOT_RIGHT_GREEN         DN_PREFIX + string("RFoot/Led/Green") + DN_POSTFIX
#define LED_FOOT_RIGHT_BLUE          DN_PREFIX + string("RFoot/Led/Blue") + DN_POSTFIX

#endif

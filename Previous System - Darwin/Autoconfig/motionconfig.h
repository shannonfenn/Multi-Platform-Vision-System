/*! @file config.h
    @brief A configuration file that controls the options for the project and the target robot
    
    This file is automatically generated by CMake. Do NOT modify this file. Seriously, don't modify
    this file. If you really need to put something here, then you want to modify ./motionconfig.in.

    @author Jason Kulk
 */
#ifndef MOTIONCONFIG_H
#define MOTIONCONFIG_H

// define variable to selectivly run motion
#define USE_MOTION_ON
#ifdef USE_MOTION_ON
    #define USE_MOTION                                           //!< this will be defined when the build is configured to include motion
#else
    #undef USE_MOTION
#endif

// Kinematics
    #define USE_MODEL_NAO_OFF
    #ifdef USE_MODEL_NAO_ON
        #define USE_MODEL_NAO
    #else
        #undef USE_MODEL_NAO
    #endif
    #define USE_MODEL_DARWIN_ON
    #ifdef USE_MODEL_DARWIN_ON
        #define USE_MODEL_DARWIN
    #else
        #undef USE_MODEL_DARWIN
    #endif



#ifdef USE_MOTION
    #define USE_HEAD_ON
    #ifdef USE_HEAD_ON
        #define USE_HEAD
    #else
        #undef USE_HEAD
    #endif
    #define USE_WALK_ON
    #ifdef USE_WALK_ON
        #define USE_WALK
    #else
        #undef USE_WALK
    #endif
    #define USE_KICK_ON
    #ifdef USE_KICK_ON
        #define USE_KICK
    #else
        #undef USE_KICK
    #endif
    #define USE_BLOCK_OFF
    #ifdef USE_BLOCK_ON
        #define USE_BLOCK
    #else
        #undef USE_BLOCK
    #endif
    #define USE_SAVE_OFF
    #ifdef USE_SAVE_ON
        #define USE_SAVE
    #else
        #undef USE_SAVE
    #endif
    #define USE_SCRIPT_OFF
    #ifdef USE_SCRIPT_ON
        #define USE_SCRIPT
    #else
        #undef USE_SCRIPT
    #endif
    #define USE_GETUP_OFF
    #ifdef USE_GETUP_ON
        #define USE_GETUP
    #else
        #undef USE_GETUP
    #endif
    #define USE_FALL_PROTECTION_ON
    #ifdef USE_FALL_PROTECTION_ON
        #define USE_FALL_PROTECTION
    #else
        #undef USE_FALL_PROTECTION
    #endif
#else
    #undef USE_HEAD
    #undef USE_WALK
    #undef USE_KICK
    #undef USE_GETUP
    #undef USE_FALL_PROTECTION
    #undef USE_MODEL_NAO
    #undef USE_MODEL_DARWIN
#endif

#endif // !MOTIONCONFIG_H


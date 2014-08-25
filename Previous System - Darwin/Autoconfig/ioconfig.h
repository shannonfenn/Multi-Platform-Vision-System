/*! @file ioconfig.h
    @brief A configuration file that controls the options for the project and the target robot
    
    This file is automatically generated by CMake. Do NOT modify this file. Seriously, don't modify
    this file. If you really need to put something here, then you want to modify ./motionconfig.in.

    @author Jason Kulk
 */
#ifndef IOCONFIG_H
#define IOCONFIG_H

// define variable to selectivly run the network
#define USE_NETWORK_ON
#ifdef USE_NETWORK_ON
    #define USE_NETWORK                                           //!< this will be defined when the build is configured to include network
#else
    #undef USE_NETWORK
#endif

#ifdef USE_NETWORK
    #define USE_NETWORK_GAMECONTROLLER_ON
    #ifdef USE_NETWORK_GAMECONTROLLER_ON
        #define USE_NETWORK_GAMECONTROLLER
    #else
        #undef USE_NETWORK_GAMECONTROLLER
    #endif

    #define USE_NETWORK_TEAMINFO_ON
    #ifdef USE_NETWORK_TEAMINFO_ON
        #define USE_NETWORK_TEAMINFO
    #else
        #undef USE_NETWORK_TEAMINFO
    #endif

    #define USE_NETWORK_JOBS_ON
    #ifdef USE_NETWORK_JOBS_ON
        #define USE_NETWORK_JOBS
    #else
        #undef USE_NETWORK_JOBS
    #endif

    #define USE_NETWORK_DEBUGSTREAM_ON
    #ifdef USE_NETWORK_DEBUGSTREAM_ON
        #define USE_NETWORK_DEBUGSTREAM
    #else
        #undef USE_NETWORK_DEBUGSTREAM
    #endif
    
    #define USE_NETWORK_SSLVISION_ON
    #ifdef USE_NETWORK_SSLVISION_ON
        #define USE_NETWORK_SSLVISION
    #else
        #undef USE_NETWORK_SSLVISION
    #endif
    
#endif

#endif // !IOCONFIG_H


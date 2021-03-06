/*! @file targetconfig.h
    @brief A configuration file that sets platform target variables.
    
    The following preprocessor variables are set here:
        - TARGET_IS_DARWIN
        - MY_OS_IS_
        - TARGET_OS_IS_
    
    This file is automatically generated by CMake. Do NOT modify this file. Seriously, don't modify
    this file. If you really need to put something here, then you want to modify ./Make/config.in.

    @author Jason Kulk
 */
#ifndef TARGETCONFIG_H
#define TARGETCONFIG_H

// define the target robotic platform
#define TARGET_IS_DARWIN            //!< Preprocessor define for determine the target platform. Will be TARGET_IS_NAO, TARGET_IS_NAOWEBOTS, TARGET_IS_CYCLOID, etc

// define the os the code is being compiled on
#define MY_OS_Linux              //!< Definition that will be MY_OS_Windows, MY_OS_Darwin or MY_OS_Linux depending on your machines OS

// define the right MY_OS_IS_XXXXXXXX
#ifdef MY_OS_Windows
    #define MY_OS_IS_WINDOWS                    //!< This will be defined only this is being configured on a Windows machine
#else
    #undef MY_OS_IS_WINDOWS
#endif
#ifdef MY_OS_Darwin
    #define MY_OS_IS_DARWIN                     //!< This will be defined only when the build is configured on a OS-X machine
#else
    #undef MY_OS_IS_DARWIN
#endif
#ifdef MY_OS_Linux
    #define MY_OS_IS_LINUX                      //!< This will be defined only when the build is configured on a Linux machine
#else
    #undef MY_OS_IS_LINUX
#endif

// now define the os of the target robotic platform
#if defined(TARGET_IS_NAOWEBOTS) or defined(TARGET_IS_DARWIN)            // If we are targeting webots, then the target os is my os
    #ifdef MY_OS_IS_WINDOWS
        #define TARGET_OS_IS_WINDOWS            //!< This will be defined if the target's os is Windows
    #else
        #undef TARGET_OS_IS_WINDOWS
    #endif
    #ifdef MY_OS_IS_DARWIN
        #define TARGET_OS_IS_DARWIN             //!< This will be defined if the target's os is OS-X
    #else
        #undef TARGET_OS_IS_DARWIN
    #endif
    #ifdef MY_OS_IS_LINUX
        #define TARGET_OS_IS_LINUX              //!< This will be defined if the target's os is Linux
    #else
        #undef TARGET_OS_IS_LINUX
    #endif
#else                                           // If we are targeting an actual robot, then the target os is always linux
    #define TARGET_OS_IS_LINUX
#endif

#endif // !TARGETCONFIG_H


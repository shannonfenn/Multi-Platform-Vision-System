# A CMake file for the layman
#   - add your source files to YOUR_SRCS
#   - to include subdirectories either
#       - put each source file in YOUR_SRCS including a *relative* path
#       - include another source.cmake for each subdirectory
#
#    Copyright (c) 2009 Jason Kulk
#    This file is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    This file is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.

IF(DEBUG)
    MESSAGE(STATUS ${CMAKE_CURRENT_LIST_FILE})
ENDIF()

# I need to prefix each file and directory with the correct path
STRING(REPLACE "/cmake/sources.cmake" "" THIS_SRC_DIR ${CMAKE_CURRENT_LIST_FILE})

INCLUDE("${THIS_SRC_DIR}/cmake/motionconfig.cmake")

########## List your source files here! ############################################
SET (YOUR_SRCS  NUMotion.cpp NUMotion.h
                NUMotionProvider.h
                FallProtection.cpp FallProtection.h
                Getup.cpp Getup.h
                Walks/WalkParameters.cpp Walks/WalkParameters.h
)

SET (YOUR_DIRS 	Tools)

IF(NUBOT_USE_MOTION_HEAD)
	LIST(APPEND YOUR_SRCS NUHead.cpp NUHead.h)
	LIST(APPEND YOUR_DIRS Head)
ENDIF()
IF(NUBOT_USE_MOTION_WALK)
	LIST(APPEND YOUR_SRCS NUWalk.cpp NUWalk.h)
	LIST(APPEND YOUR_DIRS Walks)
ENDIF()
IF(NUBOT_USE_MOTION_KICK)
	LIST(APPEND YOUR_SRCS NUKick.cpp NUKick.h)
	LIST(APPEND YOUR_DIRS Kicks)
ENDIF()
IF(NUBOT_USE_MOTION_BLOCK OR NUBOT_USE_MOTION_SAVE)
	LIST(APPEND YOUR_SRCS NUSave.cpp NUSave.h)
	LIST(APPEND YOUR_DIRS )
ENDIF()
IF(NUBOT_USE_MOTION_SCRIPT)
	LIST(APPEND YOUR_SRCS Script.cpp Script.h)
	LIST(APPEND YOUR_DIRS )
ENDIF()
####################################################################################

# Now I need to append each element to NUBOT_SRCS
FOREACH(loop_var ${YOUR_SRCS}) 
    LIST(APPEND NUBOT_SRCS "${THIS_SRC_DIR}/${loop_var}" )
ENDFOREACH(loop_var ${YOUR_SRCS})

# Do the same thing for each subdirectory in TWO steps
SET(YOUR_CMAKE_FILES )				
FOREACH(loop_var ${YOUR_DIRS}) 
    LIST(APPEND YOUR_CMAKE_FILES "${THIS_SRC_DIR}/${loop_var}/cmake/sources.cmake")
ENDFOREACH(loop_var ${YOUR_DIRS})

# We need to be careful here and this extra loop because including files will effect THIS_SRC_DIR!!!!
FOREACH(loop_var ${YOUR_CMAKE_FILES}) 
    INCLUDE(${loop_var})
ENDFOREACH(loop_var ${YOUR_CMAKE_FILES})

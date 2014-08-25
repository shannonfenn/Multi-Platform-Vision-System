DEFINES += QT_NO_DEBUG_STREAM

QMAKE_CXXFLAGS += -std=c++0x

win32{
    INCLUDEPATH += 'C:\Program Files\Boost\boost_1_52_0'
    INCLUDEPATH += 'C:\Qwt\qwt-6.0.2\src'
    DEFINES += TARGET_OS_IS_WINDOWS
}

!macx{
    !win32{
        INCLUDEPATH += /usr/include/boost/
        INCLUDEPATH += /usr/include/qwt
    }
}

macx {
    # Mac Specific Includes
    QMAKE_LFLAGS += -F/System/Library/Frameworks/CoreFoundation.framework/
    LIBS += -framework CoreFoundation -lz
    #Macports include directory
    INCLUDEPATH += '/opt/local/include'
    INCLUDEPATH += '/opt/local/include/boost/'
}

#INCLUDEPATH += ../

win32 {
    PLATFORM = win
}
macx {
    PLATFORM = mac
}
!win32 {
    !macx {
        PLATFORM = rpi
    }
}

contains(PLATFORM, "darwin") {
    message("Compiling for Darwin")
    DEFINES += TARGET_IS_DARWIN

    INCLUDEPATH += ../Autoconfig/

    HEADERS += \
        ../NUPlatform/Platforms/Darwin/DarwinCamera.h \
        VisionWrapper/datawrapperdarwin.h \
        ../Autoconfig/debug.h \
        ../Autoconfig/nubotdataconfig.h \
        VisionWrapper/visioncontrolwrapperdarwin.h \
    
    SOURCES += \
        ../NUPlatform/Platforms/Darwin/DarwinCamera.cpp \
        VisionWrapper/datawrapperdarwin.cpp \
        VisionWrapper/visioncontrolwrapperdarwin.cpp \
}

contains(PLATFORM, "pc") {
     message("Compiling for PC")
    DEFINES += TARGET_IS_PC

    INCLUDEPATH += ../Vision/NUDebug/

    HEADERS += \
        #VisionWrapper/datawrapperpc.h \
        #VisionWrapper/visioncontrolwrapperpc.h\
        ../Vision//NUDebug/debugverbosityvision.h \
        ../Vision/NUDebug/debug.h \
        ../Vision/NUDebug/nubotdataconfig.h \

    SOURCES += \
        #VisionWrapper/datawrapperpc.cpp \
        #VisionWrapper/visioncontrolwrapperpc.cpp\

    HEADERS += \
        VisionTools/pccamera.h
    SOURCES += \
        VisionTools/pccamera.cpp
}

contains(PLATFORM, "win") {
     message("Compiling for Windows")
#    DEFINES += TARGET_IS_PC
    DEFINES += TARGET_IS_WINDOWS

    INCLUDEPATH += ../Vision/NUDebug/

    HEADERS += \
        ../Vision/NUDebug/debugverbosityvision.h \
        ../Vision/NUDebug/debug.h \
        ../Vision/NUDebug/nubotdataconfig.h \

    SOURCES += \

    HEADERS += \
        ../NUPlatform/Platforms/Generic/Cameras/NUOpenCVCamera.h
    SOURCES += \
        ../NUPlatform/Platforms/Generic/Cameras/NUOpenCVCamera.cpp
}

contains(PLATFORM, "mac") {
     message("Compiling for Mac")
    DEFINES += TARGET_IS_MAC

    INCLUDEPATH += ../Vision/NUDebug/

    HEADERS += \
        ../Vision/NUDebug/debugverbosityvision.h \
        ../Vision/NUDebug/debug.h \
        ../Vision/NUDebug/nubotdataconfig.h \
        ../NUPlatform/Platforms/Generic/Cameras/NUOpenCVCamera.h \

    SOURCES += \
        ../NUPlatform/Platforms/Generic/Cameras/NUOpenCVCamera.cpp \


    LIBS += -lopencv_core -lopencv_highgui

}

contains(PLATFORM, "rpi") {
     message("Compiling for RPi")
    DEFINES += TARGET_IS_RPI

    INCLUDEPATH += ../Vision/NUDebug/

    HEADERS += \
        VisionTools/pccamera.h \
        VisionWrapper/datawrapperrpi.h \
        VisionWrapper/visioncontrolwrapperrpi.h \
        ../Vision/NUDebug/debugverbosityvision.h \
        ../Vision/NUDebug/debug.h \
        ../Vision/NUDebug/nubotdataconfig.h \

    SOURCES += \
        VisionTools/pccamera.cpp \
        VisionWrapper/datawrapperrpi.cpp \
        VisionWrapper/visioncontrolwrapperrpi.cpp\
}

INCLUDEPATH += ../

#this
HEADERS += \
    ../Vision/VisionTypes/*.h \
    ../Vision/VisionTypes/RANSACTypes/*.h \
    ../Vision/VisionTypes/VisionFieldObjects/*.h \
    ../Vision/VisionTypes/Interfaces/*.h \
    VisionWrapper/datawrappercurrent.h \
    VisionTools/classificationcolours.h \
    VisionTools/GTAssert.h \
    VisionTools/lookuptable.h \
    VisionTools/transformer.h \
    ../Vision/Modules/*.h \
    ../Vision/Modules/LineDetectionAlgorithms/*.h \
    ../Vision/Modules/BallDetectionAlgorithms/*.h \
    basicvisiontypes.h \
    valgorithm.h \
    visionblackboard.h \
    visioncontroller.h \
    visionconstants.h \
    #Threads/SaveImagesThread.h
    GenericAlgorithms/ransac.h \
    Modules/GoalDetectionAlgorithms/goaldetectorhistogram.h \
    Modules/cornerdetector.h \
    ../Tools/Math/Circle.h \
    Modules/GoalDetectionAlgorithms/goaldetectorransaccentres.h \
    Modules/GoalDetectionAlgorithms/goaldetectorransacedges.h

SOURCES += \
    Modules/balldetector.cpp \
    Modules/circledetector.cpp \
    Modules/cornerdetector.cpp \
    Modules/fieldpointdetector.cpp \
    Modules/goaldetector.cpp \
    Modules/greenhorizonch.cpp \
    Modules/linedetector.cpp \
    Modules/scanlines.cpp \
    Modules/segmentfilter.cpp \
    Modules/LineDetectionAlgorithms/linedetectorransac.cpp \
    Modules/LineDetectionAlgorithms/linedetectorsam.cpp \
    Modules/GoalDetectionAlgorithms/goaldetectorhistogram.cpp \
    Modules/GoalDetectionAlgorithms/goaldetectorransaccentres.cpp \
    Modules/GoalDetectionAlgorithms/goaldetectorransacedges.cpp \
    Modules/BallDetectionAlgorithms/balldetectordave.cpp \
    Modules/BallDetectionAlgorithms/balldetectorshannon.cpp \
    VisionTypes/colourreplacementrule.cpp \
    VisionTypes/coloursegment.cpp \
    VisionTypes/colourtransitionrule.cpp \
    VisionTypes/greenhorizon.cpp \
    VisionTypes/nupoint.cpp \
    VisionTypes/histogram1d.cpp \
    VisionTypes/quad.cpp \
    VisionTypes/segmentedregion.cpp \
    VisionTypes/RANSACTypes/ransacgoal.cpp \
    VisionTypes/VisionFieldObjects/ball.cpp \
    VisionTypes/VisionFieldObjects/centrecircle.cpp \
    VisionTypes/VisionFieldObjects/cornerpoint.cpp \
    VisionTypes/VisionFieldObjects/fieldline.cpp \
    VisionTypes/VisionFieldObjects/goal.cpp \
    VisionTypes/VisionFieldObjects/obstacle.cpp \
    VisionTypes/VisionFieldObjects/visionfieldobject.cpp\
    VisionTools/lookuptable.cpp \
    VisionTools/transformer.cpp \
    VisionTools/classificationcolours.cpp \
    visionblackboard.cpp \
    visioncontroller.cpp \
    visionconstants.cpp \
    main.cpp \
    basicvisiontypes.cpp \
    GenericAlgorithms/ransac.template \
    #../Tools/Math/Circle.cpp
    #Threads/SaveImagesThread.cpp
    Modules/obstacledetectionch.cpp

##robocup
HEADERS += \
    ../Tools/FileFormats/LUTTools.h \
    ../Tools/Optimisation/Parameter.h \
    ../Tools/Profiling/Profiler.h \
    ../Tools/Math/Line.h \
    ../Tools/Math/LSFittedLine.h \
    ../Tools/Math/Matrix.h \
    ../Tools/Math/TransformMatrices.h \
    ../Tools/Math/Vector2.h \
    ../Tools/Math/Vector3.h \
    ../Infrastructure/NUImage/NUImage.h \
    ../Infrastructure/NUImage/ColorModelConversions.h \
    ../Infrastructure/NUSensorsData/NUData.h \
    ../Infrastructure/NUSensorsData/NUSensorsData.h \
    ../Infrastructure/NUSensorsData/NULocalisationSensors.h \
    ../Infrastructure/Sensor.h \
    ../NUPlatform/NUCamera/CameraSettings.h \
    ../NUPlatform/NUCamera/NUCameraData.h \
    ../Kinematics/Horizon.h \
    ../Kinematics/Kinematics.h \
    ../Kinematics/EndEffector.h \
    ../Kinematics/Link.h \
    ../NUPlatform/NUCamera.h \
    ../Infrastructure/FieldObjects/Object.h \
    ../Infrastructure/FieldObjects/AmbiguousObject.h \
    ../Infrastructure/FieldObjects/MobileObject.h \
    ../Infrastructure/FieldObjects/StationaryObject.h \

SOURCES += \
    ../Tools/FileFormats/LUTTools.cpp \
    ../Tools/Optimisation/Parameter.cpp \
    ../Tools/Profiling/Profiler.cpp \
    ../Tools/Math/Line.cpp \
    ../Tools/Math/LSFittedLine.cpp \
    ../Tools/Math/Matrix.cpp \
    ../Tools/Math/TransformMatrices.cpp \
    ../Infrastructure/NUImage/NUImage.cpp \
    ../Infrastructure/NUData.cpp \
    ../Infrastructure/NUSensorsData/NUSensorsData.cpp \
    ../Infrastructure/NUSensorsData/NULocalisationSensors.cpp \
    ../Infrastructure/NUSensorsData/Sensor.cpp \
    ../NUPlatform/NUCamera/CameraSettings.cpp \
    ../NUPlatform/NUCamera/NUCameraData.cpp \
    ../Kinematics/Horizon.cpp \
    ../Kinematics/Kinematics.cpp \
    ../Kinematics/EndEffector.cpp \
    ../Kinematics/Link.cpp \
    ../NUPlatform/NUCamera.cpp \
    ../Infrastructure/FieldObjects/Object.cpp \
    ../Infrastructure/FieldObjects/AmbiguousObject.cpp \
    ../Infrastructure/FieldObjects/MobileObject.cpp \
    ../Infrastructure/FieldObjects/StationaryObject.cpp \

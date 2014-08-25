#############################################################################
# Makefile for building: Vision
# Project:  Vision/Vision.pro
# Template: app
#############################################################################

####### Compiler, tools and options

CC            = gcc
CXX           = g++
DEFINES       = -DTARGET_IS_RPI
#CFLAGS        = -pipe -O2 -Wall -W -D_REENTRANT $(DEFINES)
#CXXFLAGS      = -pipe -std=c++0x -O2 -Wall -W -D_REENTRANT $(DEFINES)
CFLAGS        = -pipe -O2 -Wall -Wno-unused-variable -Wno-unused-parameter -W -D_REENTRANT $(DEFINES)
CXXFLAGS      = -pipe -std=c++0x -O2 -Wall -Wno-unused-variable -Wno-unused-parameter -W -D_REENTRANT $(DEFINES)
INCPATH       = -IVision -I. -IVision/NUDebug -I/usr/X11R6/include 
LINK          = g++
LFLAGS        =
LIBS          = $(SUBLIBS)  -L/usr/lib/x86_64-linux-gnu -L/usr/X11R6/lib -L/usr/lib -lwiringPi -lrt
AR            = ar cqs
RANLIB        =
TAR           = tar -cf
COMPRESS      = gzip -9f
COPY          = cp -f
SED           = sed
COPY_FILE     = $(COPY)
COPY_DIR      = $(COPY) -r
STRIP         = strip
INSTALL_FILE  = install -m 644 -p
INSTALL_DIR   = $(COPY_DIR)
INSTALL_PROGRAM = install -m 755 -p
DEL_FILE      = rm -f
SYMLINK       = ln -f -s
DEL_DIR       = rmdir
MOVE          = mv -f
CHK_DIR_EXISTS= test -d
MKDIR         = mkdir -p

####### Output directory

OBJECTS_DIR   = ./

####### Files

SOURCES       = Vision/VisionWrapper/datawrapperrpi.cpp \
		Vision/VisionWrapper/visioncontrolwrapperrpi.cpp \
		Vision/Modules/balldetector.cpp \
		Vision/Modules/circledetector.cpp \
		Vision/Modules/cornerdetector.cpp \
		Vision/Modules/fieldpointdetector.cpp \
		Vision/Modules/goaldetector.cpp \
		Vision/Modules/greenhorizonch.cpp \
		Vision/Modules/linedetector.cpp \
		Vision/Modules/scanlines.cpp \
		Vision/Modules/segmentfilter.cpp \
		Vision/Modules/LineDetectionAlgorithms/linedetectorransac.cpp \
		Vision/Modules/LineDetectionAlgorithms/linedetectorsam.cpp \
		Vision/Modules/GoalDetectionAlgorithms/goaldetectorhistogram.cpp \
		Vision/Modules/GoalDetectionAlgorithms/goaldetectorransaccentres.cpp \
		Vision/Modules/GoalDetectionAlgorithms/goaldetectorransacedges.cpp \
		Vision/Modules/BallDetectionAlgorithms/balldetectordave.cpp \
		Vision/Modules/BallDetectionAlgorithms/balldetectorshannon.cpp \
		Vision/VisionTypes/colourreplacementrule.cpp \
		Vision/VisionTypes/coloursegment.cpp \
		Vision/VisionTypes/colourtransitionrule.cpp \
		Vision/VisionTypes/greenhorizon.cpp \
		Vision/VisionTypes/nupoint.cpp \
		Vision/VisionTypes/histogram1d.cpp \
		Vision/VisionTypes/quad.cpp \
		Vision/VisionTypes/segmentedregion.cpp \
		Vision/VisionTypes/RANSACTypes/ransacgoal.cpp \
		Vision/VisionTypes/VisionFieldObjects/ball.cpp \
		Vision/VisionTypes/VisionFieldObjects/centrecircle.cpp \
		Vision/VisionTypes/VisionFieldObjects/cornerpoint.cpp \
		Vision/VisionTypes/VisionFieldObjects/fieldline.cpp \
		Vision/VisionTypes/VisionFieldObjects/goal.cpp \
		Vision/VisionTypes/VisionFieldObjects/obstacle.cpp \
		Vision/VisionTypes/VisionFieldObjects/visionfieldobject.cpp \
		Vision/VisionTools/lookuptable.cpp \
		Vision/VisionTools/transformer.cpp \
		Vision/VisionTools/classificationcolours.cpp \
		Vision/visionblackboard.cpp \
		Vision/visioncontroller.cpp \
		Vision/visionconstants.cpp \
		Vision/main.cpp \
		Vision/basicvisiontypes.cpp \
		Vision/Modules/obstacledetectionch.cpp \
		Tools/FileFormats/LUTTools.cpp \
		Tools/Optimisation/Parameter.cpp \
		Tools/Profiling/Profiler.cpp \
		Tools/Math/Line.cpp \
		Tools/Math/LSFittedLine.cpp \
		Tools/Math/Matrix.cpp \
		Tools/Math/TransformMatrices.cpp \
		Infrastructure/NUImage/NUImage.cpp \
		Infrastructure/NUData.cpp \
		Infrastructure/NUSensorsData/NUSensorsData.cpp \
		Infrastructure/NUSensorsData/NULocalisationSensors.cpp \
		Infrastructure/NUSensorsData/Sensor.cpp \
		NUPlatform/NUCamera/CameraSettings.cpp \
		NUPlatform/NUCamera/NUCameraData.cpp \
		Kinematics/Horizon.cpp \
		Kinematics/Kinematics.cpp \
		Kinematics/EndEffector.cpp \
		Kinematics/Link.cpp \
		Infrastructure/FieldObjects/Object.cpp \
		Infrastructure/FieldObjects/AmbiguousObject.cpp \
		Infrastructure/FieldObjects/MobileObject.cpp \
		Infrastructure/FieldObjects/StationaryObject.cpp
OBJECTS       = datawrapperrpi.o \
		visioncontrolwrapperrpi.o \
		balldetector.o \
		circledetector.o \
		cornerdetector.o \
		fieldpointdetector.o \
		goaldetector.o \
		greenhorizonch.o \
		linedetector.o \
		scanlines.o \
		segmentfilter.o \
		linedetectorransac.o \
		linedetectorsam.o \
		goaldetectorhistogram.o \
		goaldetectorransaccentres.o \
		goaldetectorransacedges.o \
		balldetectordave.o \
		balldetectorshannon.o \
		colourreplacementrule.o \
		coloursegment.o \
		colourtransitionrule.o \
		greenhorizon.o \
		nupoint.o \
		histogram1d.o \
		quad.o \
		segmentedregion.o \
		ransacgoal.o \
		ball.o \
		centrecircle.o \
		cornerpoint.o \
		fieldline.o \
		goal.o \
		obstacle.o \
		visionfieldobject.o \
		lookuptable.o \
		transformer.o \
		classificationcolours.o \
		visionblackboard.o \
		visioncontroller.o \
		visionconstants.o \
		main.o \
		basicvisiontypes.o \
		obstacledetectionch.o \
		LUTTools.o \
		Parameter.o \
		Profiler.o \
		Line.o \
		LSFittedLine.o \
		Matrix.o \
		TransformMatrices.o \
		NUImage.o \
		NUData.o \
		NUSensorsData.o \
		NULocalisationSensors.o \
		Sensor.o \
		CameraSettings.o \
		NUCameraData.o \
		Horizon.o \
		Kinematics.o \
		EndEffector.o \
		Link.o \
		Object.o \
		AmbiguousObject.o \
		MobileObject.o \
		StationaryObject.o
DESTDIR       =
TARGET        = Vision

first: all
####### Implicit rules

.SUFFIXES: .o .c .cpp .cc .cxx .C

.cpp.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o "$@" "$<"

.cc.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o "$@" "$<"

.cxx.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o "$@" "$<"

.C.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o "$@" "$<"

.c.o:
	$(CC) -c $(CFLAGS) $(INCPATH) -o "$@" "$<"

####### Build rules

all: Makefile $(TARGET)

$(TARGET): $(OBJECTS)
	$(LINK) $(LFLAGS) -o $(TARGET) $(OBJECTS) $(OBJCOMP) $(LIBS)
	{ test -n "$(DESTDIR)" && DESTDIR="$(DESTDIR)" || DESTDIR=.; } && test $$(gdb --version | sed -e 's,[^0-9]\+\([0-9]\)\.\([0-9]\).*,\1\2,;q') -gt 72 && gdb --nx --batch --quiet -ex 'set confirm off' -ex "save gdb-index $$DESTDIR" -ex quit '$(TARGET)' && test -f $(TARGET).gdb-index && objcopy --add-section '.gdb_index=$(TARGET).gdb-index' --set-section-flags '.gdb_index=readonly' '$(TARGET)' '$(TARGET)' && rm -f $(TARGET).gdb-index || true


clean:
	-$(DEL_FILE) $(OBJECTS)
	-$(DEL_FILE) *~ core *.core
	-$(DEL_FILE) $(TARGET)



####### Sub-libraries

distclean: clean
	-$(DEL_FILE) $(TARGET)
	-$(DEL_FILE) Makefile


check: first

####### Compile

datawrapperrpi.o: Vision/VisionWrapper/datawrapperrpi.cpp Vision/VisionWrapper/datawrapperrpi.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o datawrapperrpi.o Vision/VisionWrapper/datawrapperrpi.cpp

visioncontrolwrapperrpi.o: Vision/VisionWrapper/visioncontrolwrapperrpi.cpp Vision/VisionWrapper/visioncontrolwrapperrpi.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o visioncontrolwrapperrpi.o Vision/VisionWrapper/visioncontrolwrapperrpi.cpp

balldetector.o: Vision/Modules/balldetector.cpp Vision/Modules/balldetector.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o balldetector.o Vision/Modules/balldetector.cpp

circledetector.o: Vision/Modules/circledetector.cpp Vision/Modules/circledetector.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o circledetector.o Vision/Modules/circledetector.cpp

cornerdetector.o: Vision/Modules/cornerdetector.cpp Vision/Modules/cornerdetector.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o cornerdetector.o Vision/Modules/cornerdetector.cpp

fieldpointdetector.o: Vision/Modules/fieldpointdetector.cpp Vision/Modules/fieldpointdetector.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o fieldpointdetector.o Vision/Modules/fieldpointdetector.cpp

goaldetector.o: Vision/Modules/goaldetector.cpp Vision/Modules/goaldetector.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o goaldetector.o Vision/Modules/goaldetector.cpp

greenhorizonch.o: Vision/Modules/greenhorizonch.cpp Vision/Modules/greenhorizonch.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o greenhorizonch.o Vision/Modules/greenhorizonch.cpp

linedetector.o: Vision/Modules/linedetector.cpp Vision/Modules/linedetector.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o linedetector.o Vision/Modules/linedetector.cpp

scanlines.o: Vision/Modules/scanlines.cpp Vision/Modules/scanlines.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o scanlines.o Vision/Modules/scanlines.cpp

segmentfilter.o: Vision/Modules/segmentfilter.cpp Vision/Modules/segmentfilter.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o segmentfilter.o Vision/Modules/segmentfilter.cpp

linedetectorransac.o: Vision/Modules/LineDetectionAlgorithms/linedetectorransac.cpp Vision/Modules/LineDetectionAlgorithms/linedetectorransac.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o linedetectorransac.o Vision/Modules/LineDetectionAlgorithms/linedetectorransac.cpp

linedetectorsam.o: Vision/Modules/LineDetectionAlgorithms/linedetectorsam.cpp Vision/Modules/LineDetectionAlgorithms/linedetectorsam.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o linedetectorsam.o Vision/Modules/LineDetectionAlgorithms/linedetectorsam.cpp

goaldetectorhistogram.o: Vision/Modules/GoalDetectionAlgorithms/goaldetectorhistogram.cpp Vision/Modules/GoalDetectionAlgorithms/goaldetectorhistogram.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o goaldetectorhistogram.o Vision/Modules/GoalDetectionAlgorithms/goaldetectorhistogram.cpp

goaldetectorransaccentres.o: Vision/Modules/GoalDetectionAlgorithms/goaldetectorransaccentres.cpp Vision/Modules/GoalDetectionAlgorithms/goaldetectorransaccentres.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o goaldetectorransaccentres.o Vision/Modules/GoalDetectionAlgorithms/goaldetectorransaccentres.cpp

goaldetectorransacedges.o: Vision/Modules/GoalDetectionAlgorithms/goaldetectorransacedges.cpp Vision/Modules/GoalDetectionAlgorithms/goaldetectorransacedges.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o goaldetectorransacedges.o Vision/Modules/GoalDetectionAlgorithms/goaldetectorransacedges.cpp

balldetectordave.o: Vision/Modules/BallDetectionAlgorithms/balldetectordave.cpp Vision/Modules/BallDetectionAlgorithms/balldetectordave.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o balldetectordave.o Vision/Modules/BallDetectionAlgorithms/balldetectordave.cpp

balldetectorshannon.o: Vision/Modules/BallDetectionAlgorithms/balldetectorshannon.cpp Vision/Modules/BallDetectionAlgorithms/balldetectorshannon.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o balldetectorshannon.o Vision/Modules/BallDetectionAlgorithms/balldetectorshannon.cpp

colourreplacementrule.o: Vision/VisionTypes/colourreplacementrule.cpp Vision/VisionTypes/colourreplacementrule.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o colourreplacementrule.o Vision/VisionTypes/colourreplacementrule.cpp

coloursegment.o: Vision/VisionTypes/coloursegment.cpp Vision/VisionTypes/coloursegment.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o coloursegment.o Vision/VisionTypes/coloursegment.cpp

colourtransitionrule.o: Vision/VisionTypes/colourtransitionrule.cpp Vision/VisionTypes/colourtransitionrule.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o colourtransitionrule.o Vision/VisionTypes/colourtransitionrule.cpp

greenhorizon.o: Vision/VisionTypes/greenhorizon.cpp Vision/VisionTypes/greenhorizon.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o greenhorizon.o Vision/VisionTypes/greenhorizon.cpp

nupoint.o: Vision/VisionTypes/nupoint.cpp Vision/VisionTypes/nupoint.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o nupoint.o Vision/VisionTypes/nupoint.cpp

histogram1d.o: Vision/VisionTypes/histogram1d.cpp Vision/VisionTypes/histogram1d.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o histogram1d.o Vision/VisionTypes/histogram1d.cpp

quad.o: Vision/VisionTypes/quad.cpp Vision/VisionTypes/quad.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o quad.o Vision/VisionTypes/quad.cpp

segmentedregion.o: Vision/VisionTypes/segmentedregion.cpp Vision/VisionTypes/segmentedregion.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o segmentedregion.o Vision/VisionTypes/segmentedregion.cpp

ransacgoal.o: Vision/VisionTypes/RANSACTypes/ransacgoal.cpp Vision/VisionTypes/RANSACTypes/ransacgoal.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o ransacgoal.o Vision/VisionTypes/RANSACTypes/ransacgoal.cpp

ball.o: Vision/VisionTypes/VisionFieldObjects/ball.cpp Vision/VisionTypes/VisionFieldObjects/ball.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o ball.o Vision/VisionTypes/VisionFieldObjects/ball.cpp

centrecircle.o: Vision/VisionTypes/VisionFieldObjects/centrecircle.cpp Vision/VisionTypes/VisionFieldObjects/centrecircle.h \
		Vision/VisionTypes/VisionFieldObjects/visionfieldobject.h \
		Vision/VisionTypes/nupoint.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o centrecircle.o Vision/VisionTypes/VisionFieldObjects/centrecircle.cpp

cornerpoint.o: Vision/VisionTypes/VisionFieldObjects/cornerpoint.cpp Vision/VisionTypes/VisionFieldObjects/cornerpoint.h \
		Vision/VisionTypes/VisionFieldObjects/visionfieldobject.h \
		Vision/VisionTypes/nupoint.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o cornerpoint.o Vision/VisionTypes/VisionFieldObjects/cornerpoint.cpp

fieldline.o: Vision/VisionTypes/VisionFieldObjects/fieldline.cpp Vision/VisionTypes/VisionFieldObjects/fieldline.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o fieldline.o Vision/VisionTypes/VisionFieldObjects/fieldline.cpp

goal.o: Vision/VisionTypes/VisionFieldObjects/goal.cpp Vision/VisionTypes/VisionFieldObjects/goal.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o goal.o Vision/VisionTypes/VisionFieldObjects/goal.cpp

obstacle.o: Vision/VisionTypes/VisionFieldObjects/obstacle.cpp Vision/VisionTypes/VisionFieldObjects/obstacle.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o obstacle.o Vision/VisionTypes/VisionFieldObjects/obstacle.cpp

visionfieldobject.o: Vision/VisionTypes/VisionFieldObjects/visionfieldobject.cpp Vision/VisionTypes/VisionFieldObjects/visionfieldobject.h \
		Vision/VisionTypes/nupoint.h \
		Vision/VisionTypes/VisionFieldObjects/beacon.h \
		Vision/VisionTypes/VisionFieldObjects/goal.h \
		Vision/VisionTypes/VisionFieldObjects/ball.h \
		Vision/VisionTypes/VisionFieldObjects/fieldline.h \
		Vision/VisionTypes/VisionFieldObjects/obstacle.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o visionfieldobject.o Vision/VisionTypes/VisionFieldObjects/visionfieldobject.cpp

lookuptable.o: Vision/VisionTools/lookuptable.cpp Vision/VisionTools/lookuptable.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o lookuptable.o Vision/VisionTools/lookuptable.cpp

transformer.o: Vision/VisionTools/transformer.cpp Vision/VisionTools/transformer.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o transformer.o Vision/VisionTools/transformer.cpp

classificationcolours.o: Vision/VisionTools/classificationcolours.cpp Vision/VisionTools/classificationcolours.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o classificationcolours.o Vision/VisionTools/classificationcolours.cpp

visionblackboard.o: Vision/visionblackboard.cpp Vision/visionblackboard.h \
		Vision/VisionWrapper/datawrappercurrent.h \
		Vision/VisionTools/lookuptable.h \
		Vision/VisionTools/transformer.h \
		Vision/basicvisiontypes.h \
		Tools/Math/Vector2.h \
		Vision/VisionTypes/coloursegment.h \
		Vision/VisionTypes/segmentedregion.h \
		Vision/VisionTypes/VisionFieldObjects/visionfieldobject.h \
		Vision/VisionTypes/nupoint.h \
		Vision/VisionTypes/VisionFieldObjects/ball.h \
		Vision/VisionTypes/VisionFieldObjects/goal.h \
		Vision/VisionTypes/VisionFieldObjects/beacon.h \
		Vision/VisionTypes/VisionFieldObjects/obstacle.h \
		Vision/VisionTypes/VisionFieldObjects/fieldline.h \
		Vision/VisionTypes/VisionFieldObjects/centrecircle.h \
		Vision/VisionTypes/VisionFieldObjects/cornerpoint.h \
		Vision/VisionTypes/greenhorizon.h \
		Vision/visionconstants.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o visionblackboard.o Vision/visionblackboard.cpp

visioncontroller.o: Vision/visioncontroller.cpp Vision/visioncontroller.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o visioncontroller.o Vision/visioncontroller.cpp

visionconstants.o: Vision/visionconstants.cpp Vision/visionconstants.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o visionconstants.o Vision/visionconstants.cpp

main.o: Vision/main.cpp Vision/VisionWrapper/visioncontrolwrapperrpi.h \
		Vision/VisionWrapper/visioncontrolwrapperrpi.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o main.o Vision/main.cpp

basicvisiontypes.o: Vision/basicvisiontypes.cpp Vision/basicvisiontypes.h \
		Tools/Math/Vector2.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o basicvisiontypes.o Vision/basicvisiontypes.cpp

obstacledetectionch.o: Vision/Modules/obstacledetectionch.cpp Vision/Modules/obstacledetectionch.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o obstacledetectionch.o Vision/Modules/obstacledetectionch.cpp

LUTTools.o: Tools/FileFormats/LUTTools.cpp Tools/FileFormats/LUTTools.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o LUTTools.o Tools/FileFormats/LUTTools.cpp

Parameter.o: Tools/Optimisation/Parameter.cpp
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o Parameter.o Tools/Optimisation/Parameter.cpp

Profiler.o: Tools/Profiling/Profiler.cpp Tools/Profiling/Profiler.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o Profiler.o Tools/Profiling/Profiler.cpp

Line.o: Tools/Math/Line.cpp Tools/Math/Line.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o Line.o Tools/Math/Line.cpp

LSFittedLine.o: Tools/Math/LSFittedLine.cpp Tools/Math/LSFittedLine.h \
		Tools/Math/Line.h \
		Tools/Math/Vector2.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o LSFittedLine.o Tools/Math/LSFittedLine.cpp

Matrix.o: Tools/Math/Matrix.cpp Tools/Math/Matrix.h \
		Tools/Math/Vector2.h \
		Tools/Math/Vector3.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o Matrix.o Tools/Math/Matrix.cpp

TransformMatrices.o: Tools/Math/TransformMatrices.cpp Tools/Math/TransformMatrices.h \
		Tools/Math/Matrix.h \
		Tools/Math/Vector2.h \
		Tools/Math/Vector3.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o TransformMatrices.o Tools/Math/TransformMatrices.cpp

NUImage.o: Infrastructure/NUImage/NUImage.cpp Infrastructure/NUImage/NUImage.h \
		Infrastructure/NUImage/Pixel.h \
		Infrastructure/NUImage/ColorModelConversions.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o NUImage.o Infrastructure/NUImage/NUImage.cpp

NUData.o: Infrastructure/NUData.cpp Infrastructure/NUData.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o NUData.o Infrastructure/NUData.cpp

NUSensorsData.o: Infrastructure/NUSensorsData/NUSensorsData.cpp Infrastructure/NUSensorsData/NUSensorsData.h \
		Infrastructure/NUSensorsData/Sensor.h \
		Infrastructure/NUSensorsData/NULocalisationSensors.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o NUSensorsData.o Infrastructure/NUSensorsData/NUSensorsData.cpp

NULocalisationSensors.o: Infrastructure/NUSensorsData/NULocalisationSensors.cpp Infrastructure/NUSensorsData/NULocalisationSensors.h \
		Infrastructure/NUSensorsData/Sensor.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o NULocalisationSensors.o Infrastructure/NUSensorsData/NULocalisationSensors.cpp

Sensor.o: Infrastructure/NUSensorsData/Sensor.cpp Infrastructure/NUSensorsData/Sensor.h \
		Tools/Math/StlVector.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o Sensor.o Infrastructure/NUSensorsData/Sensor.cpp

CameraSettings.o: NUPlatform/NUCamera/CameraSettings.cpp NUPlatform/NUCamera/CameraSettings.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o CameraSettings.o NUPlatform/NUCamera/CameraSettings.cpp

NUCameraData.o: NUPlatform/NUCamera/NUCameraData.cpp NUPlatform/NUCamera/NUCameraData.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o NUCameraData.o NUPlatform/NUCamera/NUCameraData.cpp

Horizon.o: Kinematics/Horizon.cpp Kinematics/Horizon.h \
		Kinematics/Kinematics.h \
		Kinematics/EndEffector.h \
		Kinematics/Link.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o Horizon.o Kinematics/Horizon.cpp

Kinematics.o: Kinematics/Kinematics.cpp Kinematics/Kinematics.h \
		Kinematics/EndEffector.h \
		Kinematics/Link.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o Kinematics.o Kinematics/Kinematics.cpp

EndEffector.o: Kinematics/EndEffector.cpp Kinematics/EndEffector.h \
		Kinematics/Link.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o EndEffector.o Kinematics/EndEffector.cpp

Link.o: Kinematics/Link.cpp Kinematics/Link.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o Link.o Kinematics/Link.cpp

Object.o: Infrastructure/FieldObjects/Object.cpp Infrastructure/FieldObjects/Object.h \
		Tools/Math/Vector2.h \
		Tools/Math/Vector3.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o Object.o Infrastructure/FieldObjects/Object.cpp

AmbiguousObject.o: Infrastructure/FieldObjects/AmbiguousObject.cpp Infrastructure/FieldObjects/AmbiguousObject.h \
		Infrastructure/FieldObjects/Object.h \
		Tools/Math/Vector2.h \
		Tools/Math/Vector3.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o AmbiguousObject.o Infrastructure/FieldObjects/AmbiguousObject.cpp

MobileObject.o: Infrastructure/FieldObjects/MobileObject.cpp Infrastructure/FieldObjects/MobileObject.h \
		Infrastructure/FieldObjects/Object.h \
		Tools/Math/Vector2.h \
		Tools/Math/Vector3.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o MobileObject.o Infrastructure/FieldObjects/MobileObject.cpp

StationaryObject.o: Infrastructure/FieldObjects/StationaryObject.cpp Infrastructure/FieldObjects/StationaryObject.h \
		Infrastructure/FieldObjects/Object.h \
		Tools/Math/Vector2.h \
		Tools/Math/Vector3.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o StationaryObject.o Infrastructure/FieldObjects/StationaryObject.cpp

####### Install

install:   FORCE

uninstall:   FORCE

FORCE:


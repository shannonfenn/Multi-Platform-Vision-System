#ifndef DATAWRAPPERPC_H
#define DATAWRAPPERPC_H

#include <iostream>
#include <fstream>

#include "Kinematics/Horizon.h"

#include "Vision/basicvisiontypes.h"
#include "Vision/VisionTypes/segmentedregion.h"
#include "Vision/VisionTools/lookuptable.h"
#include "Vision/VisionTypes/VisionFieldObjects/visionfieldobject.h"
#include "Vision/VisionTypes/VisionFieldObjects/ball.h"
#include "Vision/VisionTypes/VisionFieldObjects/goal.h"
#include "Vision/VisionTypes/VisionFieldObjects/obstacle.h"
#include "Vision/VisionTypes/VisionFieldObjects/fieldline.h"
#include "Vision/VisionTypes/VisionFieldObjects/centrecircle.h"
#include "Vision/VisionTypes/VisionFieldObjects/cornerpoint.h"

#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/SensorCalibration.h"
#include "NUPlatform/NUCamera/NUCameraData.h"
#include "Vision/VisionTypes/histogram1d.h"

#include "Tools/Profiling/Profiler.h"

using std::string;
using std::vector;

using namespace Vision;

#define VISION_PROFILER_ON

class DataWrapper
{
    friend class VisionController;
    friend class VisionControlWrapper;

private:
    enum PIN_MAP {
        GPIO_BALL  = 23,
        GPIO_BGOAL = 18,
        GPIO_YGOAL = 22,
        GPIO_LINE  = 17
    };

public:
    static DataWrapper* getInstance();

    //! RETRIEVAL METHODS
    NUImage* getFrame();

    float getCameraHeight() const;            //for transforms
    float getHeadPitch() const;              //for transforms
    float getHeadYaw() const;                  //for transforms
    Vector3<float> getOrientation() const;
    Vector3<double> getNeckPosition() const;
    Vector2<double> getCameraFOV() const;
    
    //! @brief Generates spoofed horizon line.
    const Horizon& getKinematicsHorizon() const;

    CameraSettings getCameraSettings() const;
    SensorCalibration getSensorCalibration() const;

    const LookUpTable& getLUT() const;
        
    //! PUBLISH METHODS
    void publish(const std::vector<const VisionFieldObject*> &visual_objects);
    void publish(const VisionFieldObject* visual_object);

    void debugPublish(const std::vector<Ball>& data);
    void debugPublish(const std::vector<Goal>& data);
    void debugPublish(const std::vector<Obstacle>& data);
    void debugPublish(const std::vector<FieldLine>& data);
    void debugPublish(const std::vector<CentreCircle>& data);
    void debugPublish(const std::vector<CornerPoint>& data);
    void debugPublish(DEBUG_ID id, const std::vector<Point>& data_points);
    void debugPublish(DEBUG_ID id, const std::vector<NUPoint>& data_points);
    void debugPublish(DEBUG_ID id, const SegmentedRegion& region);
    void debugPublish(DEBUG_ID id);
    void debugPublish(DEBUG_ID id, const NUImage *const img);
    void debugPublish(DEBUG_ID id, const std::vector<LSFittedLine>& data);
    void debugPublish(DEBUG_ID id, const std::vector<Goal>& data);

    void plotCurve(std::string name, std::vector< Point > pts);
    void plotLineSegments(std::string name, std::vector< Point > pts);
    void plotHistogram(std::string name, const Histogram1D& hist, Colour colour = yellow);

private:
    DataWrapper(std::string istrm, std::string sstrm, std::string cfg, std::string lname);
    ~DataWrapper();
    bool updateFrame();
    bool loadLUTFromFile(const std::string& fileName);
    int getNumFramesDropped() const {return numFramesDropped;}      //! @brief Returns the number of dropped frames since start.
    int getNumFramesProcessed() const {return numFramesProcessed;}  //! @brief Returns the number of processed frames since start.

private:
    static DataWrapper* instance;

    bool m_ok;

    NUImage m_current_image;
    NUSensorsData m_sensor_data;
    SensorCalibration m_sensor_calibration;

    float m_camera_height;
    float m_head_pitch;
    float m_head_yaw;
    Vector3<float> m_orientation;
    Vector3<double> m_neck_position;

    std::string configname;

    std::string LUTname;
    LookUpTable LUT;

    Horizon kinematics_horizon;

    NUCameraData m_camspecs;

    //! Used when reading from strm
    std::string streamname;
    std::ifstream imagestrm;
    bool using_sensors;
    std::string sensorstreamname;
    std::ifstream sensorstrm;

    //! Frame info
    int numFramesDropped;
    int numFramesProcessed;

    bool m_gpio;
};

#endif // DATAWRAPPERPC_H

#ifndef DATAWRAPPERDARWIN_H
#define DATAWRAPPERDARWIN_H

#include <iostream>
#include <fstream>

#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"
#include "Infrastructure/Jobs/JobList.h"
#include "Infrastructure/Jobs/VisionJobs/SaveImagesJob.h"
#include "Infrastructure/SensorCalibration.h"
#include "Kinematics/Horizon.h"

#include "Vision/VisionTools/lookuptable.h"
#include "Vision/basicvisiontypes.h"
#include "Vision/VisionTypes/segmentedregion.h"
#include "Vision/VisionTypes/histogram1d.h"
#include "Vision/VisionTypes/VisionFieldObjects/visionfieldobject.h"
#include "Vision/VisionTypes/VisionFieldObjects/ball.h"
//#include "Vision/VisionTypes/VisionFieldObjects/beacon.h"
#include "Vision/VisionTypes/VisionFieldObjects/goal.h"
#include "Vision/VisionTypes/VisionFieldObjects/obstacle.h"
#include "Vision/VisionTypes/VisionFieldObjects/fieldline.h"
#include "Vision/VisionTypes/VisionFieldObjects/centrecircle.h"
#include "Vision/VisionTypes/VisionFieldObjects/cornerpoint.h"

class NUSensorsData;
class NUActionatorsData;

class DataWrapper
{
    friend class VisionController;
    friend class VisionControlWrapper;
    
public:
    static DataWrapper* getInstance();

public:
    //! Data access interface
    
    NUImage* getFrame();

    float getCameraHeight() const;            //for transforms
    float getHeadPitch() const;              //for transforms
    float getHeadYaw() const;                  //for transforms
    Vector3<float> getOrientation() const;
    Vector3<double> getNeckPosition() const;
    Vector2<double> getCameraFOV() const;
    SensorCalibration getSensorCalibration() const;
    
    //! @brief Returns a reference to the kinematics horizon line.
    const Horizon& getKinematicsHorizon();

    const LookUpTable& getLUT() const;
        
    //! Data publish interface
    void publish(const std::vector<const VisionFieldObject*>& visual_objects);
    void publish(const VisionFieldObject* visual_object);
    //void publish(DATA_ID id, std::vector<VisionObject> data);

    void debugRefresh();
    void debugPublish(std::vector<Ball> data);
    //void debugPublish(std::vector<Beacon> data);
    void debugPublish(std::vector<Goal> data);
    void debugPublish(std::vector<Obstacle> data);
    void debugPublish(const std::vector<FieldLine>& data);
    void debugPublish(const std::vector<CentreCircle>& data);
    void debugPublish(const std::vector<CornerPoint>& data);
    void debugPublish(DEBUG_ID id, const std::vector<Point>& data_points);
    void debugPublish(DEBUG_ID id, const SegmentedRegion& region);
    void debugPublish(DEBUG_ID id) {}
    void debugPublish(DEBUG_ID id, const NUImage *const img) {}
    void debugPublish(DEBUG_ID id, const std::vector<LSFittedLine> &data);
    void debugPublish(DEBUG_ID id, const std::vector<Goal>& data) {}

    void plotCurve(std::string name, std::vector< Point > pts);
    void plotLineSegments(std::string name, std::vector< Point > pts) {}
    void plotHistogram(std::string name, const Histogram1D& hist, Colour colour = yellow) {}

    //! Control interface       
private:    
    bool updateFrame();
    void postProcess();
    bool loadLUTFromFile(const std::string& fileName);
    int getNumFramesDropped() const {return numFramesDropped;}      //! @brief Returns the number of dropped frames since start.
    int getNumFramesProcessed() const {return numFramesProcessed;}  //! @brief Returns the number of processed frames since start.
    void process(JobList* jobs);
    
    //! Vision Save Images Interface
    void saveAnImage();
    
private:
    DataWrapper();
    ~DataWrapper();
    
    static DataWrapper* instance;

    LookUpTable m_LUT;
    
    std::vector<float> m_horizon_coefficients;
    Horizon m_kinematics_horizon;
    
    //! Frame info
    double m_timestamp;
    int numFramesDropped;
    int numFramesProcessed;
    
    //! SavingImages:
    bool isSavingImages;
    bool isSavingImagesWithVaryingSettings;
    int numSavedImages;
    std::ofstream imagefile;
    std::ofstream sensorfile;
    CameraSettings currentSettings;

    SensorCalibration m_sensor_calibration;
    
    //! Shared data objects
    NUImage* current_frame;
    NUSensorsData* sensor_data;             //! pointer to shared sensor data
    NUCameraData* camera_data;
    //NUSensorsData sensor_data_copy;
    NUActionatorsData* actions;             //! pointer to shared actionators data
    FieldObjects* field_objects;            //! pointer to shared fieldobject data

    float m_camera_height;
    float m_head_pitch;
    float m_head_yaw;
    Vector3<float> m_orientation;
    Vector3<double> m_neck_position;

    std::ifstream image_stream;
};

#endif // DATAWRAPPERDARWIN_H

#ifndef DATAWRAPPERDARWIN_H
#define DATAWRAPPERDARWIN_H

#include <iostream>
#include <fstream>

#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "NUPlatform/NUCamera/NUCameraData.h"
#include "Infrastructure/SensorCalibration.h"
#include "Kinematics/Horizon.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"
#include "Infrastructure/Jobs/JobList.h"
#include "Infrastructure/Jobs/VisionJobs/SaveImagesJob.h"

#include "Vision/VisionTools/lookuptable.h"
#include "Vision/basicvisiontypes.h"
#include "Vision/VisionTypes/histogram1d.h"
#include "Vision/VisionTypes/segmentedregion.h"
#include "Vision/VisionTypes/VisionFieldObjects/visionfieldobject.h"
#include "Vision/VisionTypes/VisionFieldObjects/ball.h"
#include "Vision/VisionTypes/VisionFieldObjects/goal.h"
#include "Vision/VisionTypes/VisionFieldObjects/obstacle.h"
#include "Vision/VisionTypes/VisionFieldObjects/fieldline.h"
#include "Vision/VisionTypes/VisionFieldObjects/centrecircle.h"
#include "Vision/VisionTypes/VisionFieldObjects/cornerpoint.h"

using namespace Vision;

class DataWrapper
{
    friend class VisionController;
    friend class VisionControlWrapper;

public:
    static DataWrapper* getInstance();

    //! RETRIEVAL METHODS
    NUImage* getFrame();
    float getCameraHeight() const;              
    float getHeadPitch() const;                 
    float getHeadYaw() const;                   
    Vector3<float> getOrientation() const;
    Vector3<double> getNeckPosition() const;
    Vector2<double> getCameraFOV() const;
    const Horizon& getKinematicsHorizon();
    CameraSettings getCameraSettings() const;
    SensorCalibration getSensorCalibration() const;
    const LookUpTable& getLUT() const;
        
    //! PUBLISH METHODS
    void publish(const std::vector<const VisionFieldObject*>& visual_objects);
    void publish(const VisionFieldObject* visual_object);

    //! DEBUG LOGGING METHODS
    void debugPublish(const std::vector<Ball>& data);
    void debugPublish(const std::vector<Goal>& data);
    void debugPublish(const std::vector<Obstacle>& data);
    void debugPublish(const std::vector<FieldLine>& data);
    void debugPublish(const std::vector<CentreCircle>& data);
    void debugPublish(const std::vector<CornerPoint>& data);
    void debugPublish(DEBUG_ID id, const std::vector<Point>& data_points);
    void debugPublish(DEBUG_ID id, const SegmentedRegion& region);
    void debugPublish(DEBUG_ID id);
    void debugPublish(DEBUG_ID id, const NUImage *const img);
    void debugPublish(DEBUG_ID id, const std::vector<Goal>& data);
    void plotCurve(std::string name, std::vector< Point > pts);
    void plotLineSegments(std::string name, std::vector< Point > pts);
    void plotHistogram(std::string name, const Histogram1D& hist, Colour colour = yellow);

    //! @brief Returns the number of dropped frames since start.
    int getNumFramesDropped() const {return numFramesDropped;}
    //! @brief Returns the number of processed frames since start.
    int getNumFramesProcessed() const {return numFramesProcessed;}

private:    
    DataWrapper();
    ~DataWrapper();
    bool updateFrame();
    bool loadLUTFromFile(const std::string& fileName);

    //! External communication
    void postProcess();
    void process(JobList* jobs);
    //! Vision Save Images Interface
    void saveAnImage();
    
private:
    static DataWrapper* instance;

    LookUpTable LUT;
    
    Horizon kinematics_horizon;
    
    //! Frame count info
    int numFramesDropped;
    int numFramesProcessed;
    
    SensorCalibration sensor_calibration;
    
    //! Shared data objects
    NUImage* current_frame;
    NUSensorsData* sensor_data;             //! pointer to shared sensor data
    NUCameraData* camera_data;
    NUActionatorsData* actions;             //! pointer to shared actionators data
    FieldObjects* field_objects;            //! pointer to shared fieldobject data

    float camera_height;
    float head_pitch;
    float head_yaw;
    Vector3<float> orientation;
    Vector3<double> neck_position;

    //! Used when reading from strm
    std::ifstream imagestrm;

    double timestamp;

    //! For saving image streams
    bool isSavingImages;
    bool isSavingImagesWithVaryingSettings;
    int numSavedImages;
    std::ofstream imagefile;
    std::ofstream sensorfile;
    CameraSettings currentSettings;
};

#endif // DATAWRAPPERDARWIN_H

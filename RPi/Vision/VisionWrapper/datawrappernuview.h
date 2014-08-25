#ifndef VISIONDATAWRAPPERNUVIEW_H
#define VISIONDATAWRAPPERNUVIEW_H


#include <iostream>
#include <fstream>

#include "Kinematics/Horizon.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"
#include "Infrastructure/SensorCalibration.h"
//#include "Infrastructure/Jobs/JobList.h"
//#include "Infrastructure/Jobs/VisionJobs/SaveImagesJob.h"

#include "Vision/basicvisiontypes.h"
#include "Vision/VisionTypes/segmentedregion.h"
#include "Vision/VisionTypes/histogram1d.h"
#include "Vision/VisionTypes/VisionFieldObjects/ball.h"
//#include "Vision/VisionTypes/VisionFieldObjects/beacon.h"
#include "Vision/VisionTypes/VisionFieldObjects/goal.h"
#include "Vision/VisionTypes/VisionFieldObjects/obstacle.h"
#include "Vision/VisionTypes/VisionFieldObjects/fieldline.h"
#include "Vision/VisionTypes/VisionFieldObjects/centrecircle.h"
#include "Vision/VisionTypes/VisionFieldObjects/cornerpoint.h"

#include "Vision/VisionTools/lookuptable.h"
#include "Infrastructure/NUImage/ClassifiedImage.h"
#include "NUPlatform/NUCamera/NUCameraData.h"
#include "Vision/VisionTools/pccamera.h"

//for virtualNUbot/Qt
#include "GLDisplay.h"
#include <QObject>
#include <qwt/qwt_plot_curve.h>


//using namespace cv;

class virtualNUbot;

class DataWrapper : public QObject
{
    Q_OBJECT
    friend class VisionController;
    friend class VisionControlWrapper;
    friend class virtualNUbot;

public:
    static DataWrapper* getInstance();

    //! RETRIEVAL METHODS
    const NUImage* getFrame();


    float getCameraHeight() const;            //for transforms
    float getHeadPitch() const;              //for transforms
    float getHeadYaw() const;                  //for transforms
    Vector3<float> getOrientation() const;
    Vector3<double> getNeckPosition() const;
    Vector2<double> getCameraFOV() const;
    
    //! @brief Generates spoofed horizon line.
    const Horizon& getKinematicsHorizon();

    CameraSettings getCameraSettings();

    const LookUpTable& getLUT() const;
        
    //! PUBLISH METHODS
    void publish(const std::vector<const VisionFieldObject*> &visual_objects);
    void publish(const VisionFieldObject* visual_object);

    void debugPublish(std::vector<Ball> data)                {}
    //bool debugPublish(std::vector<Beacon> data)            {}
    void debugPublish(std::vector<Goal> data)                {}
    void debugPublish(std::vector<Obstacle> data)            {}
    void debugPublish(const std::vector<FieldLine>& data)    {}
    void debugPublish(const std::vector<CentreCircle>& data) {}
    void debugPublish(const std::vector<CornerPoint>& data)  {}
    void debugPublish(DEBUG_ID id, const std::vector<Point>& data_points);
    void debugPublish(DEBUG_ID id, const SegmentedRegion& region);
    void debugPublish(DEBUG_ID id, const NUImage *const img);
    void debugPublish(DEBUG_ID id, const std::vector<LSFittedLine> &data);
    void debugPublish(DEBUG_ID id, const std::vector<Goal>& data) {}

    void plotCurve(std::string name, std::vector< Point > pts);
    void plotLineSegments(std::string name, std::vector< Point > pts);
    void plotHistogram(std::string name, const Histogram1D& hist, Colour colour = yellow);
    
private:
    DataWrapper();
    ~DataWrapper();
    //void startImageFileGroup(std::string filename);
    bool updateFrame();
    void postProcess();
    bool loadLUTFromFile(const std::string& fileName);
    int getNumFramesDropped() const {return numFramesDropped;}      //! @brief Returns the number of dropped frames since start.
    int getNumFramesProcessed() const {return numFramesProcessed;}  //! @brief Returns the number of processed frames since start.
    void saveAnImage();
    
    //for virtualnubot
    void setRawImage(const NUImage *image);
    void setSensorData(NUSensorsData* sensors);
    void setFieldObjects(FieldObjects* fieldObjects);
    void setLUT(unsigned char* vals);
    void classifyImage(ClassifiedImage &target) const;
    void classifyPreviewImage(ClassifiedImage &target,unsigned char* temp_vals) const;
    
signals:
    void pointsUpdated(std::vector<Point> pts, GLDisplay::display disp);
    void segmentsUpdated(std::vector<std::vector<ColourSegment> > region, GLDisplay::display disp);
    void linesUpdated(std::vector<LSFittedLine> lines, GLDisplay::display disp);
    void plotUpdated(QString name, QVector<QPointF> points);
    
private:

    static DataWrapper* instance;

    const NUImage* m_current_image;

    std::string LUTname;
    LookUpTable LUT;

    std::vector<float> m_horizon_coefficients;
    Horizon m_kinematics_horizon;
    
    float m_camera_height;
    float m_head_pitch;
    float m_head_yaw;
    Vector3<float> m_orientation;
    Vector3<double> m_neck_position;

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

    //! Shared data objects
    NUSensorsData* sensor_data;             //! pointer to shared sensor data
    NUCameraData camera_data;
    NUActionatorsData* actions;             //! pointer to shared actionators data
    FieldObjects* field_objects;            //! pointer to shared fieldobject data
    SensorCalibration m_sensor_calibration;
};

#endif // VISIONDATAWRAPPERNUVIEW_H

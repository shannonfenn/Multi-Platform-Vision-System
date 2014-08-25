#include "datawrapperdarwin.h"

#include "Kinematics/Kinematics.h"
#include "debug.h"
#include "nubotdataconfig.h"
#include "debugverbosityvision.h"


#include "Vision/VisionTypes/coloursegment.h"
#include "Vision/basicvisiontypes.h"
#include "Vision/visionconstants.h"

#include "Infrastructure/NUBlackboard.h"
#include "Infrastructure/Jobs/CameraJobs/ChangeCameraSettingsJob.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"
#include "NUPlatform/NUActionators/NUSounds.h"

DataWrapper* DataWrapper::instance = 0;

DataWrapper::DataWrapper()
{
    numFramesDropped = 0;
    numFramesProcessed = 0;
    numSavedImages = 0;
    loadLUTFromFile(std::string(DATA_DIR) + std::string("default.lut"));
    Blackboard->lookForBall = true;
    Blackboard->lookForGoals = true;
    isSavingImages = false;
    isSavingImagesWithVaryingSettings = false;

    debug << "Loading from: " << std::string(CONFIG_DIR) + std::string("VisionOptions.cfg") << std::endl;
    VisionConstants::loadFromFile(std::string(CONFIG_DIR) + std::string("VisionOptions.cfg"));

    std::string sen_calib_name = std::string(CONFIG_DIR) + std::string("SensorCalibration.cfg");

    debug << "opening sensor calibration config: " << sen_calib_name << std::endl;
    if( ! sensor_calibration.ReadSettings(sen_calib_name)) {
        errorlog << "DataWrapper::DataWrapper() - failed to load sensor calibration: " << sen_calib_name << ". Using default values." << std::endl;
        sensor_calibration = SensorCalibration();
    }

    std::cout << "Opening image stream." << std::endl;
    image_stream.open( (std::string(DATA_DIR) + std::string("image.strm")).c_str() );

    current_frame = new NUImage();
}

DataWrapper::~DataWrapper()
{
    delete current_frame;
}

DataWrapper* DataWrapper::getInstance()
{
    if (instance == 0)
        instance = new DataWrapper();
    return instance;
}

/**
*   @brief Fetches the next frame.
*/
NUImage* DataWrapper::getFrame()
{
    return current_frame;
}

/*! @brief Retrieves the horizon data and builds a Horizon and returns it.
*   @return kinematics_horizon A reference to the kinematics horizon line.
*   @note This method has a chance of retrieving an invalid line, in this case
*           the old line is returned with the "exists" flag set to false.
*/
const Horizon& DataWrapper::getKinematicsHorizon()
{
    #if VISION_WRAPPER_VERBOSITY > 1
        debug << "DataWrapper::getKinematicsHorizon() - Begin" << std::endl;
    #endif
    std::vector<float> horizon_coefficients;
    if(sensor_data->getHorizon(horizon_coefficients)) {
        #if VISION_WRAPPER_VERBOSITY > 1
            debug << "DataWrapper::getKinematicsHorizon() - success" << std::endl;
        #endif
        kinematics_horizon.setLine(horizon_coefficients.at(0), horizon_coefficients.at(1), horizon_coefficients.at(2));
        kinematics_horizon.exists = true;
    }
    else {
        #if VISION_WRAPPER_VERBOSITY > 1
            debug << "DataWrapper::getKinematicsHorizon() - failed" << std::endl;
        #endif
        kinematics_horizon.setLineFromPoints(Point(0, current_frame->getHeight()), Point(current_frame->getWidth(), current_frame->getHeight()));
        kinematics_horizon.exists = false;
    }
    
    return kinematics_horizon;
}

//! @brief Retrieves the camera height returns it.
float DataWrapper::getCameraHeight() const
{
    return camera_height;
}

//! @brief Retrieves the camera pitch returns it.
float DataWrapper::getHeadPitch() const
{
    return head_pitch;
}

//! @brief Retrieves the camera yaw returns it.
float DataWrapper::getHeadYaw() const
{
    return head_yaw;
}

//! @brief Retrieves the body pitch returns it.
Vector3<float> DataWrapper::getOrientation() const
{
    return orientation;
}

//! @brief Returns the neck position snapshot.
Vector3<double> DataWrapper::getNeckPosition() const
{
    return neck_position;
}

Vector2<double> DataWrapper::getCameraFOV() const
{
    return Vector2<double>(camera_data->m_horizontalFov, camera_data->m_verticalFov);
}

//! @brief Returns camera settings.
CameraSettings DataWrapper::getCameraSettings() const
{
    return CameraSettings();
}

SensorCalibration DataWrapper::getSensorCalibration() const
{
    return sensor_calibration;
}

/*! @brief Returns a reference to the stored Lookup Table
*   @return LUT A reference to the current LUT
*/
const LookUpTable& DataWrapper::getLUT() const
{
    return LUT;
}

void DataWrapper::publish(const std::vector<const VisionFieldObject*>& visual_objects)
{
    for(int i=0; i<visual_objects.size(); i++) {
        visual_objects.at(i)->addToExternalFieldObjects(field_objects, timestamp);
    }
}

void DataWrapper::publish(const VisionFieldObject* visual_object)
{
    visual_object->addToExternalFieldObjects(field_objects, timestamp);
}

void DataWrapper::debugPublish(const std::vector<Ball>& data) 
{
    #if VISION_WRAPPER_VERBOSITY > 1
        debug << "DataWrapper::debugPublish - DEBUG_ID = " << getIDName(DBID_BALLS) << std::endl;
        for(Ball ball : data) {
            debug << "DataWrapper::debugPublish - Ball = " << ball << std::endl;
        }
    #endif
}

void DataWrapper::debugPublish(const std::vector<Goal>& data) 
{
    #if VISION_WRAPPER_VERBOSITY > 1
        debug << "DataWrapper::debugPublish - DEBUG_ID = " << getIDName(DBID_GOALS) << std::endl;
        for(Goal post : data) {
            debug << "DataWrapper::debugPublish - Goal = " << post << std::endl;
        }
    #endif
}

void DataWrapper::debugPublish(const std::vector<Obstacle>& data) 
{
    #if VISION_WRAPPER_VERBOSITY > 1
        debug << "DataWrapper::debugPublish - DEBUG_ID = " << getIDName(DBID_OBSTACLES) << std::endl;
        for(Obstacle obst : data) {
            debug << "DataWrapper::debugPublish - Obstacle = " << obst << std::endl;
        }
    #endif
}

void DataWrapper::debugPublish(const std::vector<FieldLine>& data)
{
    #if VISION_WRAPPER_VERBOSITY > 2
        debug << "DataWrapper::debugPublish - DEBUG_ID = " << getIDName(id) << std::endl;
        for(const FieldLine& l : data) {
            debug << "DataWrapper::debugPublish - Line = ";
            l.printLabel(debug);
            debug << std::endl;
        }
    #endif
}

void DataWrapper::debugPublish(const std::vector<CentreCircle>& data)
{
    #if VISION_WRAPPER_VERBOSITY > 2
        debug << "DataWrapper::debugPublish - DEBUG_ID = " << getIDName(id) << std::endl;
        for(const CentreCircle& c : data) {
            debug << "DataWrapper::debugPublish - CentreCircle = ";
            debug << c << std::endl;
        }
    #endif
}

void DataWrapper::debugPublish(const std::vector<CornerPoint>& data)
{
    #if VISION_WRAPPER_VERBOSITY > 2
        debug << "DataWrapper::debugPublish - DEBUG_ID = " << getIDName(id) << std::endl;
        for(const CornerPoint& c : data) {
            debug << "DataWrapper::debugPublish - CornerPoint = ";
            debug << c << std::endl;
        }
    #endif
}

void DataWrapper::debugPublish(DEBUG_ID id, const std::vector<Point>& data_points)
{
    #if VISION_WRAPPER_VERBOSITY > 2
        debug << "DataWrapper::debugPublish - DEBUG_ID = " << getIDName(id) << std::endl;
        debug << "\t" << id << std::endl;
        debug << "\t" << data_points << std::endl;
    #endif
}

void DataWrapper::debugPublish(DEBUG_ID id, const SegmentedRegion& region)
{
    #if VISION_WRAPPER_VERBOSITY > 2
        debug << "DataWrapper::debugPublish - DEBUG_ID = " << getIDName(id) << std::endl;
        for(const vector<ColourSegment>& line : region.getSegments()) {
            if(region.getDirection() == VisionID::HORIZONTAL)
                debug << "y: " << line.front().getStart().y << std::endl;
            else
                debug << "x: " << line.front().getStart().x << std::endl;
            for(const ColourSegment& seg : line) {
                debug << "\t" << seg;
            }
        }
    #endif
}

void DataWrapper::debugPublish(DEBUG_ID id, NUImage const* const img)
{
}

void DataWrapper::debugPublish(DEBUG_ID id, const std::vector<Goal>& data)
{
}

void DataWrapper::plotCurve(std::string name, std::vector< Point > pts)
{
}

void DataWrapper::plotLineSegments(std::string name, std::vector< Point > pts)
{
}

void DataWrapper::plotHistogram(std::string name, const Histogram1D& hist, Colour colour)
{
}

/*! @brief Updates the held information ready for a new frame.
*   Gets copies of the actions and sensors pointers from the blackboard and
*   gets a new image from the blackboard. Updates framecounts.
*   @return Whether the fetched data is valid.
*/
bool DataWrapper::updateFrame()
{
    static int frame_no = 0;
    #if VISION_WRAPPER_VERBOSITY > 1
        debug << "DataWrapper::updateFrame() - Begin" << std::endl;
    #endif

    actions = Blackboard->Actions;
    sensor_data = Blackboard->Sensors;
    camera_data = Blackboard->CameraSpecs;
    field_objects = Blackboard->Objects;
    
    if (current_frame != NULL and Blackboard->Image->GetTimestamp() - timestamp > 40)
        numFramesDropped++;
    numFramesProcessed++;
//    current_frame = Blackboard->Image;
    try {
        image_stream >> *current_frame;
        std::cout << "Frame updated. " << ++frame_no << std::endl;
    }
    catch(std::exception& e) {
        std::cout << "Frame read failed." << std::endl;
        return false;
    }
    
    if (current_frame == NULL || sensor_data == NULL || actions == NULL || field_objects == NULL)
    {
        #if VISION_WRAPPER_VERBOSITY > 1
            debug << "DataWrapper::updateFrame(): null reference from BB" << std::endl;
        #endif
        // keep object times updated.
        if(field_objects && sensor_data)
        {
            field_objects->preProcess(sensor_data->GetTimestamp());
            field_objects->postProcess(sensor_data->GetTimestamp());
        }
        return false;
    }
    timestamp = current_frame->GetTimestamp();
    //succesful
    field_objects->preProcess(timestamp);
#if VISION_WRAPPER_VERBOSITY > 1
    debug << "Frames dropped: " << numFramesDropped << std::endl;
#endif

    vector<float> orientation(3, 0);

    //update kinematics snapshot
    if(!sensor_data->getCameraHeight(camera_height))
        errorlog << "DataWrapperDarwin - updateFrame() - failed to get camera height from NUSensorsData" << std::endl;
    if(!sensor_data->getPosition(NUSensorsData::HeadPitch, head_pitch))
        errorlog << "DataWrapperDarwin - updateFrame() - failed to get head pitch from NUSensorsData" << std::endl;
    if(!sensor_data->getPosition(NUSensorsData::HeadYaw, head_yaw))
        errorlog << "DataWrapperDarwin - updateFrame() - failed to get head yaw from NUSensorsData" << std::endl;
    if(!sensor_data->getOrientation(orientation))
        errorlog << "DataWrapperDarwin - updateFrame() - failed to get orientation from NUSensorsData" << std::endl;
    orientation = Vector3<float>(orientation.at(0), orientation.at(1), orientation.at(2));

    vector<float> left, right;
    if(sensor_data->get(NUSensorsData::LLegTransform, left) and sensor_data->get(NUSensorsData::RLegTransform, right))
    {
        neck_position = Kinematics::CalculateNeckPosition(Matrix4x4fromVector(left), Matrix4x4fromVector(right), sensor_calibration.m_neck_position_offset);
    }
    else
    {
        errorlog << "DataWrapperDarwin - updateFrame() - failed to get left or right leg transforms from NUSensorsData" << std::endl;
        // Default in case kinemtaics not available. Base height of darwin.
        neck_position = Vector3<double>(0.0, 0.0, 39.22);
    }

    return current_frame->getWidth() > 0 && current_frame->getHeight() > 0;
}

/**
*   @brief loads the colour look up table
*   @param filename The filename for the LUT stored on disk
*/
bool DataWrapper::loadLUTFromFile(const std::string& fileName)
{
    #if VISION_WRAPPER_VERBOSITY > 1
    debug << "DataWrapper::loadLUTFromFile() - " << fileName << std::endl;
    #endif
    return LUT.loadLUTFromFile(fileName);
}

/**
*   @brief Post processes field objects with image timestamp.
*/
void DataWrapper::postProcess()
{
    if (current_frame != NULL && field_objects != NULL)
    {
        field_objects->postProcess(timestamp);
    }
}

/**
*   @brief Processes saving images jobs.
*   @param jobs The current JobList
*   @note Taken from original vision system
*/
void DataWrapper::process(JobList* jobs)
{
    #if VISION_WRAPPER_VERBOSITY > 1
        debug  << "DataWrapper::Process - Begin" << std::endl;
    #endif
    static std::list<Job*>::iterator it;     // the iterator over the motion jobs
    
    for (it = jobs->vision_begin(); it != jobs->vision_end();)
    {
        if ((*it)->getID() == Job::VISION_SAVE_IMAGES)
        {   
            #if VISION_WRAPPER_VERBOSITY > 1
                debug << "DataWrapper::process(): Processing a save images job." << std::endl;
            #endif
            static SaveImagesJob* job;
            job = (SaveImagesJob*) (*it);
            if(isSavingImages != job->saving()) {
                //if the job changes the saving images state
                if(job->saving() == true) {
                    //we weren't saving and now we've started
                    currentSettings = current_frame->getCameraSettings();
                    if (!imagefile.is_open())
                        imagefile.open((std::string(DATA_DIR) + std::string("image.strm")).c_str());
                    if (!sensorfile.is_open())
                        sensorfile.open((std::string(DATA_DIR) + std::string("sensor.strm")).c_str());
                    actions->add(NUActionatorsData::Sound, sensor_data->CurrentTime, NUSounds::START_SAVING_IMAGES);
                }
                else {
                    //we were saving and now we've finished
                    imagefile.flush();
                    sensorfile.flush();

                    ChangeCameraSettingsJob* newJob  = new ChangeCameraSettingsJob(currentSettings);
                    jobs->addCameraJob(newJob);
                    actions->add(NUActionatorsData::Sound, sensor_data->CurrentTime, NUSounds::STOP_SAVING_IMAGES);
                }
            }
            isSavingImages = job->saving();
            isSavingImagesWithVaryingSettings = job->varyCameraSettings();
            it = jobs->removeVisionJob(it);
        }
        else 
        {
            ++it;
        }
    }
}

/**
*   @brief Saves an image and the current sensor data to the associated streams.
*/
void DataWrapper::saveAnImage()
{
    #if VISION_WRAPPER_VERBOSITY > 2
        debug << "DataWrapper::SaveAnImage(). Starting..." << std::endl;
    #endif

    if (!imagefile.is_open())
        imagefile.open((std::string(DATA_DIR) + std::string("image.strm")).c_str());
    if (!sensorfile.is_open())
        sensorfile.open((std::string(DATA_DIR) + std::string("sensor.strm")).c_str());

    if (imagefile.is_open() and numSavedImages < 2500)
    {
        if(sensorfile.is_open())
        {
            sensorfile << (*sensor_data) << std::flush;
        }
        NUImage buffer;
        buffer.cloneExisting(*current_frame);
        imagefile << buffer;
        numSavedImages++;
    }
    #if VISION_WRAPPER_VERBOSITY > 1
        debug << "DataWrapper::SaveAnImage(). Finished" << std::endl;
    #endif
}
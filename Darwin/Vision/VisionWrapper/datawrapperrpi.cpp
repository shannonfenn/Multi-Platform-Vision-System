#include "datawrapperrpi.h"
//#include "Infrastructure/NUImage/ColorModelConversions.h"
#include "Kinematics/Kinematics.h"
#include "debug.h"
#include "nubotdataconfig.h"

#include "Vision/VisionTypes/coloursegment.h"
#include "Vision/visionconstants.h"

//for controlling GPIO pins
#include <wiringPi.h>

DataWrapper* DataWrapper::instance = 0;

DataWrapper::DataWrapper(std::string istrm, std::string sstrm, std::string cfg, std::string lname)
{
    m_ok = true;
    string cam_spec_name = string(CONFIG_DIR) + string("CameraSpecs.cfg");
    string sen_calib_name = string(CONFIG_DIR) + string("SensorCalibration.cfg");

    debug << "opening camera config: " << cam_spec_name << std::endl;
    if( ! m_camspecs.LoadFromConfigFile(cam_spec_name.c_str())) {
        errorlog << "DataWrapper::DataWrapper() - failed to load camera specifications: " << cam_spec_name << std::endl;
        m_ok = false;
    }

    debug << "opening sensor calibration config: " << sen_calib_name << std::endl;
    if( ! m_sensor_calibration.ReadSettings(sen_calib_name)) {
        errorlog << "DataWrapper::DataWrapper() - failed to load sensor calibration: " << sen_calib_name << ". Using default values." << std::endl;
        m_sensor_calibration = SensorCalibration();
    }

    kinematics_horizon.setLine(0, 1, 0);
    numFramesDropped = numFramesProcessed = 0;

    streamname = istrm;
    debug << "openning image stream: " << streamname << std::endl;
    imagestrm.open(streamname.c_str());

    using_sensors = !sstrm.empty();
    sensorstreamname = sstrm;
    if(m_ok && using_sensors) {
        debug << "openning sensor stream: " << sensorstreamname << std::endl;
        sensorstrm.open(sensorstreamname.c_str());
        if(!sensorstrm.is_open()) {
            std::cerr << "Error : Failed to read sensors from: " << sensorstreamname << " defaulting to sensors off." << std::endl;
            using_sensors = false;
        }
    }

    if(!imagestrm.is_open()) {
        errorlog << "DataWrapper::DataWrapper() - failed to load stream: " << streamname << std::endl;
        m_ok = false;
    }

    configname = cfg;
    debug << "config: " << configname << std::endl;
    VisionConstants::loadFromFile(configname);

    LUTname = lname;
    if(!loadLUTFromFile(LUTname)){
        errorlog << "DataWrapper::DataWrapper() - failed to load LUT: " << LUTname << std::endl;
        m_ok = false;
    }
    m_gpio = (wiringPiSetupSys() != -1);
    if(m_gpio) {
        system("gpio -g mode 17 out");
        system("gpio -g mode 18 out");
        system("gpio -g mode 22 out");
        system("gpio -g mode 23 out");
        system("gpio export 17 out");
        system("gpio export 18 out");
        system("gpio export 22 out");
        system("gpio export 23 out");
//        pinMode(17,OUTPUT);
//        pinMode(18,OUTPUT);
//        pinMode(22,OUTPUT);
//        pinMode(23,OUTPUT);
    }
    else {
        std::cerr << "Failed to setup wiringPi - GPIO pins unavailable" << std::endl;
    }
}

DataWrapper::~DataWrapper()
{
    imagestrm.close();
    if(using_sensors)
        sensorstrm.close();
}

DataWrapper* DataWrapper::getInstance()
{
    return instance;
}

/**
*   @brief Fetches the next frame from the webcam.
*/
NUImage* DataWrapper::getFrame()
{
    return &m_current_image;
}

//! @brief Retrieves the camera height returns it.
float DataWrapper::getCameraHeight() const
{
    return m_camera_height;
}

//! @brief Retrieves the camera pitch returns it.
float DataWrapper::getHeadPitch() const
{
    return m_head_pitch;
}

//! @brief Retrieves the camera yaw returns it.
float DataWrapper::getHeadYaw() const
{
    return m_head_yaw;
}

//! @brief Retrieves the body pitch returns it.
Vector3<float> DataWrapper::getOrientation() const
{
    return m_orientation;
}

//! @brief Returns the neck position snapshot.
Vector3<double> DataWrapper::getNeckPosition() const
{
    return m_neck_position;
}

Vector2<double> DataWrapper::getCameraFOV() const
{
    return Vector2<double>(m_camspecs.m_horizontalFov, m_camspecs.m_verticalFov);
}

//! @brief Returns spoofed kinecv::Matics horizon.
const Horizon& DataWrapper::getKinematicsHorizon() const
{
    return kinematics_horizon;
}

//! @brief Returns camera settings.
CameraSettings DataWrapper::getCameraSettings() const
{
    return CameraSettings();
}

SensorCalibration DataWrapper::getSensorCalibration() const
{
    return m_sensor_calibration;
}

const LookUpTable& DataWrapper::getLUT() const
{
    return LUT;
}

void DataWrapper::publish(const std::vector<const VisionFieldObject*> &visual_objects)
{
    //std::cout << visual_objects.size() << " visual objects seen" << std::endl;
}

void DataWrapper::publish(const VisionFieldObject* visual_object)
{
//    std::cout << "Visual object (" << visual_object->getName() << ") seen at " << visual_object->getLocationPixels() << std::endl;

    if( isGoal(visual_object->getID()) )
    {
        digitalWrite(18, HIGH);
    }
    else if( visual_object->getID() == BALL )
    {
        digitalWrite(17, HIGH);
    }
    else if( visual_object->getID() == FIELDLINE )
    {
        digitalWrite(23, HIGH);
    }
    else if( visual_object->getID() == OBSTACLE )
    {
        digitalWrite(22, HIGH);
    }

    #if VISION_WRAPPER_VERBOSITY > 0
    visual_object->printLabel(debug);
    debug << std::endl;
    #endif
}

void DataWrapper::debugPublish(const std::vector<Ball>& data) {
//    for(Ball b : data) {
//        if(b.isValid())
//            std::cout << "Ball: " << b.getLocationPixels() << " " <<  b.getRadius() << std::endl;
//    }
}

void DataWrapper::debugPublish(const std::vector<CentreCircle>& data)
{
//    for(CentreCircle c : data) {
//        //need to change to display as ellipse - but for now just centre
//        if(c.isValid())
//            std::cout << "CC: " << c.getLocationPixels() << " " <<  c.getGroundRadius() << std::endl;
//    }
}

void DataWrapper::debugPublish(const std::vector<CornerPoint>& data)
{
//    for(CornerPoint c : data) {
//        if(c.isValid())
//            std::cout << "Corner: " << c.getLocationPixels() << std::endl;
//    }
}

void DataWrapper::debugPublish(const std::vector<Goal>& data)
{
//    for(Goal g : data)
//    {
//        if(g.isValid())
//        {
//            std::cout << "Goal: " << g.getLocationPixels() << " " << g.getScreenSize() << std::endl;
//        }
//    }
}

void DataWrapper::debugPublish(const std::vector<Obstacle>& data)
{
//    for(Obstacle o : data) {
//        if(o.isValid()) {
//            std::cout << "Obstacle: " << o.getLocationPixels() << " " << o.getScreenSize() << std::endl;
//        }
//    }
}

void DataWrapper::debugPublish(const std::vector<FieldLine> &data)
{
//    for(FieldLine l : data) {
//        if(l.isValid()) {
//            std::cout << "Line: " << l.getEndPoints() << std::endl;
//        }
//    }
}

void DataWrapper::debugPublish(DEBUG_ID id, const std::vector<Point> &data_points)
{
}

//! Outputs debug data to the appropriate external interface
void DataWrapper::debugPublish(DEBUG_ID id, const SegmentedRegion& region)
{
}

void DataWrapper::debugPublish(DEBUG_ID id, NUImage const* const img)
{
    //for all images
}

void DataWrapper::debugPublish(DEBUG_ID id, const std::vector<LSFittedLine>& data)
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

bool DataWrapper::updateFrame()
{
    digitalWrite (17, LOW);
    digitalWrite (18, LOW);
    digitalWrite (22, LOW);
    digitalWrite (23, LOW);

    if(m_ok) {
       VisionConstants::loadFromFile(configname);
        if(!imagestrm.is_open()) {
            errorlog << "No image stream - " << streamname << std::endl;
            return false;
        }
        if(using_sensors && !sensorstrm.is_open()) {
            errorlog << "No sensor stream - " << sensorstreamname << std::endl;
            return false;
        }
        try {
            imagestrm >> m_current_image;
        }
        catch(std::exception& e) {
            return false;
        }
        if(using_sensors) {
            try {
                sensorstrm >> m_sensor_data;
            }
            catch(std::exception& e){
                errorlog << "Sensor stream error: " << e.what() << std::endl;
                return false;
            }
        }

        //overwrite sensor horizon if using sensors
        std::vector<float> hor_data;
        if(using_sensors && m_sensor_data.getHorizon(hor_data)) {
            kinematics_horizon.setLine(hor_data.at(0), hor_data.at(1), hor_data.at(2));
        }

        //update kinematics snapshot
        if(using_sensors) {

            vector<float> orientation(3, 0);

            if(!m_sensor_data.getCameraHeight(m_camera_height))
                errorlog << "DataWrapperQt - updateFrame() - failed to get camera height from NUSensorsData" << std::endl;
            if(!m_sensor_data.getPosition(NUSensorsData::HeadPitch, m_head_pitch))
                errorlog << "DataWrapperQt - updateFrame() - failed to get head pitch from NUSensorsData" << std::endl;
            if(!m_sensor_data.getPosition(NUSensorsData::HeadYaw, m_head_yaw))
                errorlog << "DataWrapperQt - updateFrame() - failed to get head yaw from NUSensorsData" << std::endl;
            if(!m_sensor_data.getOrientation(orientation))
                errorlog << "DataWrapperQt - updateFrame() - failed to get orientation from NUSensorsData" << std::endl;
            m_orientation = Vector3<float>(orientation.at(0), orientation.at(1), orientation.at(2));

            vector<float> left, right;
            if(m_sensor_data.get(NUSensorsData::LLegTransform, left) and m_sensor_data.get(NUSensorsData::RLegTransform, right))
            {
                m_neck_position = Kinematics::CalculateNeckPosition(Matrix4x4fromVector(left), Matrix4x4fromVector(right), m_sensor_calibration.m_neck_position_offset);
            }
            else
            {
                errorlog << "DataWrapperQt - updateFrame() - failed to get left or right leg transforms from NUSensorsData" << std::endl;
                // Default in case kinemtaics not available. Base height of darwin.
                m_neck_position = Vector3<double>(0.0, 0.0, 39.22);
            }
        }
        else {
            m_camera_height = m_head_pitch = m_head_yaw = 0;
            m_orientation = Vector3<float>(0,0,0);
            m_neck_position = Vector3<double>(0.0, 0.0, 39.22);
        }

        numFramesProcessed++;

        return m_current_image.getHeight() > 0 && m_current_image.getWidth() > 0;
    }
    return false;
}

/**
*   @brief loads the colour look up table
*   @param filename The filename for the LUT stored on disk
*   @note Taken from original vision system
*/
bool DataWrapper::loadLUTFromFile(const std::string& fileName)
{
    return LUT.loadLUTFromFile(fileName);
}

#include "visioncontrolwrapperrpi.h"

VisionControlWrapper* VisionControlWrapper::instance = 0;

VisionControlWrapper* VisionControlWrapper::getInstance()
{
    if(!instance)
        instance = new VisionControlWrapper();
    else
        instance->wrapper = DataWrapper::getInstance();
    return instance;
}

VisionControlWrapper::VisionControlWrapper()
{
    wrapper = new DataWrapper("/home/pi/nubot/image.strm", "", "/home/pi/nubot/Config/rpi/VisionOptions.cfg", "/home/pi/nubot/default.lut");
    DataWrapper::instance = wrapper;
}

int VisionControlWrapper::runFrame()
{
    if(!wrapper->updateFrame()) {
        #if VISION_WRAPPER_VERBOSITY > 1
            debug << "VisionControlWrapper::runFrame() - updateFrame() failed" << std::endl;
        #endif
        return -1;  //failure - do not run vision
    }
    return controller.runFrame(true, true, true, true);
}

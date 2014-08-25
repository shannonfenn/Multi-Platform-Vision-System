#ifndef CONTROLWRAPPER_H
#define CONTROLWRAPPER_H

#include "Vision/visioncontroller.h"
#include "Vision/VisionWrapper/datawrapperrpi.h"

class NUSensorsData;
class NUActionatorsData;

class VisionControlWrapper
{
public:
    static VisionControlWrapper* getInstance();
    
    //! Vision Control Interface
    int runFrame();
    
private:
    VisionControlWrapper();
    
    static VisionControlWrapper* instance;
    
    VisionController controller;
    DataWrapper* data_wrapper;
};

#endif // CONTROLWRAPPER_H

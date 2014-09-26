#ifndef DATAWRAPPERCURRENT_H
#define DATAWRAPPERCURRENT_H

#ifdef TARGET_IS_RPI
    #include "Vision/VisionWrapper/datawrapperrpi.h"
#else
    #include "Vision/VisionWrapper/datawrapperdarwin.h"
#endif

#endif // DATAWRAPPERCURRENT_H

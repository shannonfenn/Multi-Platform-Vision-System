#ifndef DATAWRAPPERCURRENT_H
#define DATAWRAPPERCURRENT_H

#ifdef TARGET_IS_RPI
    #include "Vision/VisionWrapper/datawrapperrpi.h"
#elif TARGET_IS_PC || TARGET_IS_MAC || TARGET_IS_WINDOWS
    //#include "Vision/VisionWrapper/datawrapperpc.h"
    #include "Vision/VisionWrapper/datawrapperqt.h"
#elif TARGET_IS_NUVIEW
    #include "Vision/VisionWrapper/datawrappernuview.h"
#elif TARGET_IS_TRAINING
    #include "Vision/VisionWrapper/datawrappertraining.h"
#else
    #include "Vision/VisionWrapper/datawrapperdarwin.h"
#endif

#endif // DATAWRAPPERCURRENT_H

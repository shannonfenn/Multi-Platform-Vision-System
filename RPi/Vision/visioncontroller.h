/**
*       @name   VisionController
*       @file   visioncontroller.h
*       @brief  Controller for vision blackboard.
*       @author Shannon Fenn
*       @author David Budden
*       @date   02/03/12
*
*/

#ifndef VISIONCONTROLLER_H
#define VISIONCONTROLLER_H


#include "Vision/visionblackboard.h"
#include "Vision/VisionWrapper/datawrappercurrent.h"
#include "Vision/Modules/segmentfilter.h"
#include "Vision/Modules/linedetector.h"
#include "Vision/Modules/fieldpointdetector.h"
#include "Vision/Modules/cornerdetector.h"
#include "Vision/Modules/goaldetector.h"
#include "Vision/Modules/balldetector.h"
#include "debugverbosityvision.h"

class VisionController
{
public:
    VisionController();
    ~VisionController();
    /**
    *   @brief Runs the vision system for a single frame.
    *   @return A status indication of the execution of the frame.
    */
    int runFrame(bool lookForBall, bool lookForGoals, bool lookForFieldPoints, bool lookForObstacles);

private:
//! VARIABLES
    DataWrapper* m_data_wrapper;               //! @variable Reference to singleton Wrapper for vision system
    VisionBlackboard* m_blackboard;     //! @variable Reference to singleton Blackboard for vision system
    SegmentFilter m_segment_filter;       //! @variable Segment filter object for pre-classification filtering

    BallDetector* m_ball_detector_shannon;
    BallDetector* m_ball_detector_dave;

    FieldPointDetector* m_field_point_detector;

    GoalDetector* m_goal_detector_hist;
    GoalDetector* m_goal_detector_ransac_edges;

    LineDetector* m_line_detector_sam;
    LineDetector* m_line_detector_ransac;
    CornerDetector m_corner_detector;
    CircleDetector m_circle_detector;


#ifdef VISION_PROFILER_ON
    std::ofstream m_profiling_stream;
#endif
};

#endif // VISIONCONTROLLER_H

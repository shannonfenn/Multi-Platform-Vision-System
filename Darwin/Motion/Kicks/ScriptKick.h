#ifndef SCRIPTKICK_H
#define SCRIPTKICK_H

#include "Motion/NUKick.h"
#include "Tools/Math/Rectangle.h"

class MotionScript;

class ScriptKick : public NUKick
{
public:
    ScriptKick(NUWalk* walk, NUSensorsData* data, NUActionatorsData* actions);
    ~ScriptKick();
    void doKick();
    void kickToPoint(const std::vector<float>& position, const std::vector<float>& target);
    
    virtual void stop();
    virtual void kill();
    bool isActive();
    bool isUsingHead();
    bool isUsingArms();
    bool isUsingLegs();

    bool requiresHead();
    bool requiresArms();
    bool requiresLegs();

private:
    void loadKickParameters();

protected:
    MotionScript* m_left_kick_script;
    MotionScript* m_right_kick_script;
    MotionScript* m_side_left_kick_script;
    MotionScript* m_side_right_kick_script;
    double m_script_start_time;
    unsigned int m_current_index;
    double m_script_end_time;
    MotionScript* m_current_script;
    std::vector<std::vector<float> > m_joints;
    std::vector<std::vector<float> > m_gains;
    Rectangle m_left_kick_area;
    Rectangle m_right_kick_area;
    Rectangle m_side_left_kick_area;
    Rectangle m_side_right_kick_area;
    float m_kick_enable_time;

};

#endif // SCRIPTKICK_H

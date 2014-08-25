#include "ScriptKick.h"
#include "debug.h"
#include "debugverbositynumotion.h"
#include "Motion/Tools/MotionScript.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Motion/NUWalk.h"

//for configs
#include "nubotdataconfig.h"
#include <boost/algorithm/string.hpp>

ScriptKick::ScriptKick(NUWalk* walk, NUSensorsData* data, NUActionatorsData* actions): NUKick(walk, data, actions)
{
    m_left_kick_script = new MotionScript("KickLeft");
    m_right_kick_script = new MotionScript("KickRight");
    m_side_left_kick_script = new MotionScript("SideKickLeft");
    m_side_right_kick_script = new MotionScript("SideKickRight");
    m_current_script = NULL;
    m_script_start_time = -1;
    m_script_end_time = -1;
    m_kick_enable_time = 0;
    loadKickParameters();
}

/*! @brief Kills the kick module
 */
void ScriptKick::kill()
{
    m_current_script = NULL;
    m_script_start_time = -1;
    m_kick_ready = false;
    m_kick_enabled = false;
    setArmEnabled(false, false);
    setHeadEnabled(false);
    m_kicking_leg = noLeg;
    m_script_end_time = -1;
}

void ScriptKick::stop()
{
    #if DEBUG_NUMOTION_VERBOSITY > 3
    debug << "Kick stop called. Finishing Kick." << std::endl;
    #endif
    if(m_script_start_time == -1 or m_current_script == NULL or
        m_data->CurrentTime > m_current_script->timeFinished()) {
        kill();
    }
    //stopHead();
    //stopArms();
    //stopLegs();
    //m_kick_enabled = false;
    //m_current_script = NULL;
}

ScriptKick::~ScriptKick()
{
    delete m_left_kick_script;
    delete m_right_kick_script;
    m_current_script = NULL;
}

void ScriptKick::loadKickParameters()
{
    //defaults to guarantee at least old performance in case of bad file
    float x_min_right_forward = 5.0f;
    float x_max_right_forward = 11.0f;
    float y_min_right_forward = -3.2f;
    float y_max_right_forward = -9.5f;

    float x_min_left_forward = 5.0f;
    float x_max_left_forward = 11.0f;
    float y_min_left_forward = 3.2f;
    float y_max_left_forward = 9.5f;

    float x_min_right_side = 5.0f;
    float x_max_right_side = 11.0f;
    float y_min_right_side = -3.2f;
    float y_max_right_side = -6.5f;

    float x_min_left_side= 5.0f;
    float x_max_left_side = 11.0f;
    float y_min_left_side = 3.2f;
    float y_max_left_side = 6.5f;

    std::string filename = CONFIG_DIR + std::string("kickboxes.cfg");
    std::ifstream input(filename.c_str());
    std::string id_str;
    if(input.good()) {
        float xmin, xmax, ymin, ymax;
        while(input.good()) {
            // read in the rule name
            getline(input, id_str, ':');
            boost::trim(id_str);
            boost::to_lower(id_str);

            input.ignore(30, '(');
            input >> xmin;
            input.ignore(10, ',');
            input >> xmax;
            input.ignore(10, ',');
            input >> ymin;
            input.ignore(10, ',');
            input >> ymax;
            input.ignore(10, ')');

            // ignore the rest of the line
            input.ignore(128, '\n');
            input.peek();               //trigger eofbit being set in the case of this being the last rule

            if(id_str.compare("rightforward") == 0) {
                x_min_right_forward = xmin;
                x_max_right_forward = xmax;
                y_min_right_forward = ymin;
                y_max_right_forward = ymax;
            }
            else if(id_str.compare("leftforward") == 0) {
                x_min_left_forward = xmin;
                x_max_left_forward = xmax;
                y_min_left_forward = ymin;
                y_max_left_forward = ymax;
            }
            else if(id_str.compare("rightside") == 0) {
                x_min_right_side = xmin;
                x_max_right_side = xmax;
                y_min_right_side = ymin;
                y_max_right_side = ymax;
            }
            else if(id_str.compare("leftside") == 0) {
                x_min_left_side= xmin;
                x_max_left_side = xmax;
                y_min_left_side = ymin;
                y_max_left_side = ymax;
            }
            else
                errorlog << "ScriptKick::loadKickParameters - invalid kick name: " << id_str << std::endl;
        }
    }
    else {
        errorlog << "ScriptKick::loadKickParameters - failed to load kickboxes.cfg" << std::endl;
    }

    std::cout << " x_min_right_forward = " << x_min_right_forward << std::endl;
    std::cout << " x_max_right_forward = " << x_max_right_forward << std::endl;
    std::cout << " y_min_right_forward = " << y_min_right_forward << std::endl;
    std::cout << " y_max_right_forward = " << y_max_right_forward << std::endl;

    std::cout << " x_min_left_forward = " << x_min_left_forward << std::endl;
    std::cout << " x_max_left_forward = " << x_max_left_forward << std::endl;
    std::cout << " y_min_left_forward = " << y_min_left_forward << std::endl;
    std::cout << " y_max_left_forward = " << y_max_left_forward << std::endl;

    std::cout << " x_min_right_side = " << x_min_right_side << std::endl;
    std::cout << " x_max_right_side = " << x_max_right_side << std::endl;
    std::cout << " y_min_right_side = " << y_min_right_side << std::endl;
    std::cout << " y_max_right_side = " << y_max_right_side << std::endl;

    std::cout << " x_min_left_side = " << x_min_left_side << std::endl;
    std::cout << " x_max_left_side = " << x_max_left_side << std::endl;
    std::cout << " y_min_left_side = " << y_min_left_side << std::endl;
    std::cout << " y_max_left_side = " << y_max_left_side << std::endl;

    m_right_kick_area = Rectangle(x_min_right_forward, x_max_right_forward, y_min_right_forward, y_max_right_forward);
    m_left_kick_area = Rectangle(x_min_left_forward, x_max_left_forward, y_min_left_forward, y_max_left_forward); //HACK: move right kick box three cm to right
    m_side_right_kick_area = Rectangle(x_min_right_side, x_max_right_side, y_min_right_side, y_max_right_side); //HACK: kick box less wide for side kicks
    m_side_left_kick_area = Rectangle(x_min_left_side, x_max_left_side, y_min_left_side, y_max_left_side);
    return;
}


bool ScriptKick::isActive()
{
    /*std::cout << "m_kick_ready: " << m_kick_ready << std::endl;
    std::cout << "m_kick_enabled: " << m_kick_enabled << std::endl;
    std::cout << "m_kicking_leg: " << m_kicking_leg << std::endl;
    std::cout << "m_current_script: " << m_current_script << std::endl;
    std::cout << "m_script_start_time: " << m_script_start_time << std::endl;
    if(m_current_script != NULL)
    {
        std::cout << "m_data->CurrentTime: " << m_data->CurrentTime << std::endl;
        std::cout << "m_current_script->timeFinished(): " << m_current_script->timeFinished() << std::endl;    
    }*/
    //double check the weird conditions
    if (m_kick_enabled and m_current_script == NULL)
        kill();

    if(m_data->CurrentTime - m_kick_enable_time  > 16000)
        stop();
    
    return m_kick_enabled;
    
}

bool ScriptKick::isUsingHead()
{
    if(m_current_script != NULL)
    {
        return m_data->CurrentTime > m_current_script->timeFinishedWithHead();
    }
    else
    {
        return false;
    }
}

bool ScriptKick::isUsingArms()
{
    if(m_current_script != NULL)
    {
        return m_data->CurrentTime > std::max(m_current_script->timeFinishedWithLArm(),m_current_script->timeFinishedWithRArm()+100);
    }
    else
    {
        return false;
    }
}

/*! @brief Returns true if a script uses the legs */
bool ScriptKick::isUsingLegs()
{
    if(m_current_script != NULL)
    {
        return m_data->CurrentTime > std::max(m_current_script->timeFinishedWithLLeg(),m_current_script->timeFinishedWithRLeg()+100);
    }
    else
    {
        return false;
    }
}

bool ScriptKick::requiresHead()
{
    if(m_current_script != NULL)
    {
        return m_current_script->usesHead();
    }
    else
    {
        return false;
    }
}

bool ScriptKick::requiresArms()
{
    if(m_current_script != NULL)
    {
        return m_current_script->usesLArm() or m_current_script->usesRArm();
    }
    else
    {
        return false;
    }
}

bool ScriptKick::requiresLegs()
{
    if(m_current_script != NULL)
    {
        return m_current_script->usesLLeg() or m_current_script->usesRLeg();
    }
    else
    {
        return false;
    }
}

void ScriptKick::doKick()
{
    if(m_current_script and m_kick_enabled and m_kick_ready)
    {
        static std::vector<float> nu_nextLeftArmJoints(m_actions->getSize(NUActionatorsData::LArm), 0.0f);   // Left Arm
        static std::vector<float> nu_nextRightArmJoints(m_actions->getSize(NUActionatorsData::RArm), 0.0f);  // Right Arm
        static std::vector<float> nu_nextLeftLegJoints(m_actions->getSize(NUActionatorsData::LLeg), 0.0f);   // Left Leg
        static std::vector<float> nu_nextRightLegJoints(m_actions->getSize(NUActionatorsData::RLeg), 0.0f);  // Right Leg
        static std::vector<float> nu_nextHeadJoints(m_actions->getSize(NUActionatorsData::Head), 0.0f);  // Right Leg

        static std::vector<float> nu_nextLeftArmGains(m_actions->getSize(NUActionatorsData::LArm), 0.0f);   // Left Arm
        static std::vector<float> nu_nextRightArmGains(m_actions->getSize(NUActionatorsData::RArm), 0.0f);  // Right Arm
        static std::vector<float> nu_nextLeftLegGains(m_actions->getSize(NUActionatorsData::LLeg), 0.0f);   // Left Leg
        static std::vector<float> nu_nextRightLegGains(m_actions->getSize(NUActionatorsData::RLeg), 0.0f);  // Right Leg
        static std::vector<float> nu_nextHeadGains(m_actions->getSize(NUActionatorsData::Head), 0.0f);  // Right Leg
        if((m_walk == NULL or !m_walk->isActive()) and (m_script_start_time == -1))   // Either we have no walk, or we want it to be inactive.
        {
            m_current_index = 0;
            m_script_start_time = m_data->CurrentTime;
            m_script_end_time = m_data->CurrentTime + m_current_script->m_times.back().back();

            m_joints = std::vector<std::vector<float> > (m_current_script->m_times.back().size(), std::vector<float>());
            m_gains = std::vector<std::vector<float> > (m_current_script->m_times.back().size(), std::vector<float>());

            unsigned int totalJoints = m_current_script->m_positions.size();
            unsigned int totalFrames = m_current_script->m_times.back().size();

            for(unsigned int j = 0; j < totalFrames; ++j)
            {
                for(unsigned int i = 0; i < totalJoints; ++i)
                {
                    if(m_current_script->m_positions.at(i).empty())
                    {
                        m_joints[j].push_back(0.0f);
                        m_gains[j].push_back(50.0f);
                    }
                    else
                    {
                        m_joints[j].push_back(m_current_script->m_positions.at(i).at(j));
                        m_gains[j].push_back(m_current_script->m_gains.at(i).at(j));
                    }
                }
            }

            std::vector<float> joints = m_joints.at(m_current_index);
            std::vector<float> gains = m_gains.at(m_current_index);

            float targetTime = m_script_start_time + m_current_script->m_times.back().at(m_current_index);

            if(m_current_script->usesHead())
            {
                nu_nextHeadJoints.assign(joints.begin(), joints.begin()+2);
                nu_nextHeadGains.assign(gains.begin(), gains.begin()+2);
                m_actions->add(NUActionatorsData::Head, targetTime, nu_nextHeadJoints, nu_nextHeadGains);
            }
            if(m_current_script->usesLArm())
            {
                nu_nextLeftArmJoints.assign(joints.begin()+2, joints.begin()+5);
                nu_nextLeftArmGains.assign(gains.begin()+2, gains.begin()+5);
                m_actions->add(NUActionatorsData::LArm, targetTime, nu_nextLeftArmJoints, nu_nextLeftArmGains);
            }
            if(m_current_script->usesRArm())
            {
                nu_nextRightArmJoints.assign(joints.begin()+5, joints.begin()+8);
                nu_nextRightArmGains.assign(gains.begin()+5, gains.begin()+8);
                m_actions->add(NUActionatorsData::RArm, targetTime, nu_nextRightArmJoints, nu_nextRightArmGains);
            }
            if(m_current_script->usesLLeg())
            {
                nu_nextLeftLegJoints.assign(joints.begin()+8, joints.begin()+14);
                nu_nextLeftLegGains.assign(gains.begin()+8, gains.begin()+14);
                m_actions->add(NUActionatorsData::LLeg, targetTime, nu_nextLeftLegJoints, nu_nextLeftLegGains);
            }
            if(m_current_script->usesRLeg())
            {
                nu_nextRightLegJoints.assign(joints.begin()+14, joints.begin()+20);
                nu_nextRightLegGains.assign(gains.begin()+14, gains.begin()+20);
                m_actions->add(NUActionatorsData::RLeg, targetTime, nu_nextRightLegJoints, nu_nextRightLegGains);
            }

        }
        else if(m_current_index < m_current_script->m_times.back().size()-1)
        {
            double currentScriptTime = m_data->CurrentTime - m_script_start_time;
            double nextTime = m_current_script->m_times.back().at(m_current_index);
            if(nextTime <= currentScriptTime)
            {
                ++m_current_index;
                std::vector<float> joints = m_joints.at(m_current_index);
                std::vector<float> gains = m_gains.at(m_current_index);

                float targetTime = m_script_start_time + m_current_script->m_times.back().at(m_current_index);

                if(m_current_script->usesHead())
                {
                    nu_nextHeadJoints.assign(joints.begin(), joints.begin()+2);
                    nu_nextHeadGains.assign(gains.begin(), gains.begin()+2);
                    m_actions->add(NUActionatorsData::Head, targetTime, nu_nextHeadJoints, nu_nextHeadGains);
                }
                if(m_current_script->usesLArm())
                {
                    nu_nextLeftArmJoints.assign(joints.begin()+2, joints.begin()+5);
                    nu_nextLeftArmGains.assign(gains.begin()+2, gains.begin()+5);
                    m_actions->add(NUActionatorsData::LArm, targetTime, nu_nextLeftArmJoints, nu_nextLeftArmGains);
                }
                if(m_current_script->usesRArm())
                {
                    nu_nextRightArmJoints.assign(joints.begin()+5, joints.begin()+8);
                    nu_nextRightArmGains.assign(gains.begin()+5, gains.begin()+8);
                    m_actions->add(NUActionatorsData::RArm, targetTime, nu_nextRightArmJoints, nu_nextRightArmGains);
                }
                if(m_current_script->usesLLeg())
                {
                    nu_nextLeftLegJoints.assign(joints.begin()+8, joints.begin()+14);
                    nu_nextLeftLegGains.assign(gains.begin()+8, gains.begin()+14);
                    m_actions->add(NUActionatorsData::LLeg, targetTime, nu_nextLeftLegJoints, nu_nextLeftLegGains);
                }
                if(m_current_script->usesRLeg())
                {
                    nu_nextRightLegJoints.assign(joints.begin()+14, joints.begin()+20);
                    nu_nextRightLegGains.assign(gains.begin()+14, gains.begin()+20);
                    m_actions->add(NUActionatorsData::RLeg, targetTime, nu_nextRightLegJoints, nu_nextRightLegGains);
                }

            }
        }
        else
        {
            m_current_script = NULL;
            m_script_start_time = -1;
            m_kick_ready = false;
            m_kick_enabled = false;
            setArmEnabled(false, false);
            setHeadEnabled(false);
            m_kicking_leg = noLeg;
            m_script_end_time = -1;
        }
    }
    //std::cout << "dokick is called" << std::endl;
//    if(m_current_script and m_kick_enabled and m_kick_ready and (m_script_start_time == -1))    // Check if there is a script ready, that has not been started.
//    {
//        if(m_walk == NULL or !m_walk->isActive())   // Either we have no walk, or we want it to be inactive.
//        {
//            //std::cout << "Kick Beginning." << std::endl;
//            m_current_script->play(m_data, m_actions);
//            m_script_start_time = m_data->CurrentTime;
//            m_script_end_time = m_current_script->timeFinished();
//        }
        
//    }
//    #if DEBUG_NUMOTION_VERBOSITY > 3
//    debug << "Current Time: " << m_data->CurrentTime << std::endl;
//    debug << "Walk: " << m_walk->isActive()<< std::endl;
//    debug << "Script will finish at: " << m_current_script->timeFinished() << std::endl;
//    #endif

//    if((m_script_end_time != -1) and (m_data->CurrentTime > m_script_end_time))
//    {
//        //std::cout << "Kick Complete. " << m_data->CurrentTime << ", " << m_current_script->timeFinished() << ", " << m_script_start_time << std::endl;
//        // Kick has finished
//        m_current_script = NULL;
//        m_script_start_time = -1;
//        m_kick_ready = false;
//        m_kick_enabled = false;
//        setArmEnabled(false, false);
//        setHeadEnabled(false);
//        m_kicking_leg = noLeg;
//        m_script_end_time = -1;
//    }
    return;
}

void ScriptKick::kickToPoint(const std::vector<float> &position, const std::vector<float> &target)
{
    bool kick_begin = false;

    if(isActive()) return;

    float ball_x = position[0];
    float ball_y = position[1];

    float target_x = target[0];
    float target_y = target[1];

    double theta = atan2(target_y - ball_y, target_x - ball_x);

    float angle_margin = mathGeneral::PI / 4.0f; //triggers sidekick too often with -45 deg to 45 deg front kick zone

    //float angle_margin = mathGeneral::PI / 8.0f + mathGeneral::PI / 4.0f; //trigger front kick from -67.5 deg to 67.5 deg

    /*if(fabs(theta) > angle_margin)
    {
        //std::cout << "Angle Too Large: " << theta << std::endl;
        return;
    }*/
    //std::cout << theta << std::endl; //angle for debugging kick boxes / kick triggering
    // Ball is in position for left kick.

    if((/*true or*/ theta > angle_margin and m_side_left_kick_area.PointInside(ball_x, ball_y)) and
       m_side_left_kick_script->isValid())
    {
        kick_begin = true;
        m_kicking_leg = rightLeg;
        m_current_script = m_side_left_kick_script;
        #if DEBUG_NUMOTION_VERBOSITY > 3
        #endif
    }
    else if(theta <= angle_margin and theta >= -angle_margin and
            m_left_kick_area.PointInside(ball_x, ball_y) and m_left_kick_script->isValid())
    {
        kick_begin = true;
        m_kicking_leg = leftLeg;
        m_current_script = m_left_kick_script;
        #if DEBUG_NUMOTION_VERBOSITY > 3
        debug << "leftfront: " << theta << std::endl;
        #endif
    }
    else if(theta < -angle_margin and m_side_right_kick_area.PointInside(ball_x, ball_y) and
            m_side_right_kick_script->isValid())
    {
        kick_begin = true;
        m_kicking_leg = leftLeg;
        m_current_script = m_side_right_kick_script;
        #if DEBUG_NUMOTION_VERBOSITY > 3
        debug << "rightside: " << theta << std::endl;
        #endif
    }
    else if(theta >= -angle_margin and theta <= angle_margin
            and m_right_kick_area.PointInside(ball_x, ball_y) and m_right_kick_script->isValid())
    {
        kick_begin = true;
        m_kicking_leg = rightLeg;
        m_current_script = m_right_kick_script;
        #if DEBUG_NUMOTION_VERBOSITY > 3
        debug << "rightfront: " << theta << std::endl;
        #endif
    }
    else
    {
        //std::cout << "No kick available for position: (" << ball_x << ", " << ball_y << ")" << std::endl;
        return;
    }

    if(kick_begin)
    {
        m_walk->stop();
        m_kick_ready = true;
        m_kick_enabled = true;
        setArmEnabled(true, true);
        setHeadEnabled(true);
        m_kick_enable_time = m_data->CurrentTime;
        //std::cout << "Starting kick: " << toString(m_kicking_leg) << std::endl;
    }
    return;
}


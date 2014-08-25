/*! @file Script.h
    @brief Declaration of a ball blocking class
 
    @class Script
    @brief A module to block the ball using the legs (suitable for both goal keeper and field player)
 
    @author Jason Kulk
 
  Copyright (c) 2010 Jason Kulk
 
    This file is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This file is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with NUbot.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef _SCRIPT_H_
#define _SCRIPT_H_

class NUSensorsData;
class NUActionatorsData;
class ScriptJob;
class NUWalk;
#include "Motion/NUMotionProvider.h"
#include "Motion/Tools/MotionScript.h"

class Script : public NUMotionProvider
{
public:
    Script(NUWalk* walk, NUSensorsData* data, NUActionatorsData* actions);
    ~Script();
    
    void stop();
    void stopHead();
    void stopArms();
    void stopLegs();
    void kill();
    
    bool isActive();
    bool isUsingHead();
    bool isUsingArms();
    bool isUsingLegs();
    bool isReady();
    
    bool requiresHead();
    bool requiresArms();
    bool requiresLegs();
    
    void process(NUSensorsData* data, NUActionatorsData* actions);
    void process(ScriptJob* job);
private:
public:
private:
    NUWalk* m_walk;
    double m_script_start_time;
    bool m_script_pending;
    MotionScript m_script;
};

#endif


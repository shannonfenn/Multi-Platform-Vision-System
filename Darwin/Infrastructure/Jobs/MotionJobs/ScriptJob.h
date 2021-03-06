/*! @file ScriptJob.h
    @brief Declaration of ScriptJob class.
 
    @class ScriptJob
    @brief A class to encapsulate jobs issued for the save module.
 
    @author Jason Kulk
 
  Copyright (c) 2009 Jason Kulk
 
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

#ifndef SCRIPTJOB_H
#define SCRIPTJOB_H

#include "../MotionJob.h"
#include "Motion/Tools/MotionScript.h"
#include <vector>
#include <string>


class ScriptJob : public MotionJob
{
public:
    ScriptJob(double time, const MotionScript& script);
    ScriptJob(double time, const std::string& name);
    ScriptJob(double time, std::istream& input);
    ~ScriptJob();
    
    void getScript(double& time, MotionScript& script);
    std::string& getName();
    
    virtual void summaryTo(std::ostream& output);
    virtual void csvTo(std::ostream& output);
    
    friend std::ostream& operator<<(std::ostream& output, const ScriptJob& job);
    friend std::ostream& operator<<(std::ostream& output, const ScriptJob* job);
protected:
    virtual void toStream(std::ostream& output) const;
private:
    std::string m_name;
    MotionScript m_script;                  // the motion script attached to the job
};

#endif


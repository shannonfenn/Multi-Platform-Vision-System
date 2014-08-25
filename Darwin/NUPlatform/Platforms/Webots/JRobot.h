/*! @file JRobot.h
    @brief Declaration of a JRobot (A slightly extended version of Webots' Robot) class
    @author Jason Kulk
 
    @class actionator_t
    @brief A JRobot (A slightly extended version of Webots' Robot) class
 
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

#ifndef JROBOT_H
#define JROBOT_H

#include "JServo.h"
#include <webots/Robot.hpp>

namespace webots
{
    class JRobot : public Robot 
    {
    public:
        JRobot();
        virtual ~JRobot();
    protected:
        virtual Servo *createServo(const std::string &name) const;
        
    };
}

#endif


/*! @file NUActionators.h
    @brief Declaration of a base actuator class for robot manipulation

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

#ifndef NUACTIONATORS_H
#define NUACTIONATORS_H

class NUActionatorsData;
class NUSoundThread;

#include <vector>
#include <string>
using namespace std;

class NUActionators
{
public:
    NUActionators();
    virtual ~NUActionators();
    
    void process(NUActionatorsData* data);
    NUActionatorsData* getNUActionatorsData();
protected:
    virtual void copyToHardwareCommunications() = 0;
    virtual void copyToSound();
protected:
    double m_current_time;
    NUActionatorsData* m_data;
    NUSoundThread* m_sound_thread;
};

#endif


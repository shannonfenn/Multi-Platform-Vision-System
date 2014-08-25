/*! @file WalkParameters.h
    @brief Declaration of WalkParameters class
 
    @class WalkParameters
    @brief A module to store walk parameters
 
    Each walk engine must implement a function to get and save its parameters to a WalkParameters object.
    The walk parameters are separated into:
        - arm gains (0 to 100%)
        - torso gains (0 to 100%)
        - leg gains (0 to 100%)
        - parameters (walk engine parameters)
 
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

#ifndef WALKPARAMETERS_H
#define WALKPARAMETERS_H

#include "Tools/Optimisation/Parameter.h"

#include <vector>
#include <string>
#include <iostream>
#include <sstream>
using namespace std;

class WalkParameters
{
public:
    WalkParameters();
    WalkParameters(const string& name);
    WalkParameters(const string& name, const vector<float>& maxspeeds, const vector<float>& maxaccels, const vector<Parameter>& parameters, const vector<vector<float> >& armgains, const vector<vector<float> >& torsogains, const vector<vector<float> >& leggains);
    ~WalkParameters();
    
    // get methods
    vector<float> getAsVector();
    vector<Parameter> getAsParameters();
    string& getName();
    vector<float>& getMaxSpeeds();
    vector<float>& getMaxAccelerations();
    vector<Parameter>& getParameters();
    vector<vector<float> >& getArmGains();
    vector<vector<float> >& getTorsoGains();
    vector<vector<float> >& getLegGains();
    
    // set methods
    void set(const vector<float>& data);
    void setName(const string& name);
    void setMaxSpeeds(const vector<float>& maxspeeds);
    void setMaxAccelerations(const vector<float>& maxaccels);
    void setParameters(const vector<Parameter>& parameters);
    void setArmGains(const vector<vector<float> >& armgains);
    void setTorsoGains(const vector<vector<float> >& torsogains);
    void setLegGains(const vector<vector<float> >& leggains);
    
    // display methods
    void summaryTo(ostream& output);
    void csvTo(ostream& output);
    
    // serialisation
    friend ostream& operator<< (ostream& output, const WalkParameters& p_walkparameters);
    friend ostream& operator<< (ostream& output, const WalkParameters* p_walkparameters);
    friend istream& operator>> (istream& input, WalkParameters& p_walkparameters);
    friend istream& operator>> (istream& input, WalkParameters* p_walkparameters);
    void save();
    void saveAs(const string& name);
    void load(const string& name);
    
    size_t size() const;
private:
    void setGains(vector<vector<float> >& gains, unsigned int& numgains, const vector<vector<float> >& newgains);
public:
private:
    string m_name;                             //!< the name of the walk parameter set
    vector<float> m_max_speeds;                //!< stores the maximum speeds (x,y,theta) allowed by the walk engine
    vector<float> m_max_accelerations;         //!< stores the maximum accelerations (x,y,theta) allowed by the walk engine
    
    vector<Parameter> m_parameters;            //!< stores the parameters for the walk engine
    
    vector<vector<float> > m_arm_gains;        //!< stores the arm gains for a walk
    unsigned int m_num_arm_gains;              //!< stores the total number of arm gains
    vector<vector<float> > m_torso_gains;      //!< stores the torso gains for a walk
    unsigned int m_num_torso_gains;            //!< stores the total number of torso gains
    vector<vector<float> > m_leg_gains;        //!< stores the leg gains for a walk
    unsigned int m_num_leg_gains;              //!< stores the total number of leg gains
};

#endif


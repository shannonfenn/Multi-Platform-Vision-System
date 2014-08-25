/*! @file DictionaryApproximator.h
    @brief Uses a discretised lookup table derived from the continuous input std::vectors. Used in: MRLAgent.

    @author Josiah Walker

 Copyright (c) 2012 Josiah Walker

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


#ifndef DICTAPPROXIMATOR_H
#define DICTAPPROXIMATOR_H

#include <sstream>
#include <fstream>
#include <vector>

#include <map>
#include <string>
#include <vector>
#include <iostream>
#include "ApproximatorInterface.h"



class DictionaryApproximator: public ApproximatorInterface {

private:
    int tileMultiplier,numInputs,numOutputs;
    std::map<std::string,float> approximator;
    float getValue(std::vector<float> const& observations,int action);
    float setValue(std::vector<float> const& observations,int action,float value);
    std::string getRepresentation(std::vector<float> const& observations,int action);
    
public:
    /*! @brief numberOfHiddens represents the tileMultiplier variable. This variable controls the resolution of the discretisation of the lookup table.
      Higher numberOfHiddens gives a higher resolution.
    */
    virtual void initialiseApproximator(int numberOfInputs, int numberOfOutputs, int numberOfHiddens, float max_parameter_range = 1);
    
    virtual void doLearningEpisode(std::vector< std::vector<float> > const& observations, std::vector< std::vector<float> > const& values, float stepSize=0.1, int iterations=1);
    
    virtual std::vector<float> getValues(std::vector<float> const& observations);
    
    virtual void saveApproximator(std::string agentName);
    
    virtual void loadApproximator(std::string agentName);
    
    std::map<std::string,float>* getMap();

    DictionaryApproximator():ApproximatorInterface(){}
    
};

#endif

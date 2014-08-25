/*! @file ApproximatorInterface.h
    @brief Standard interface for a function approximator mapping vector of floats to a vector of floats.

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

#ifndef APPROXIMATORINTERFACE_H
#define APPROXIMATORINTERFACE_H

#include <vector>
#include<string>


class ApproximatorInterface {
    
public:
    ApproximatorInterface(){ save_location = "nubot/Config/Darwin/RLearning/";}

    virtual void initialiseApproximator(int numberOfInputs, int numberOfOutputs, int numberOfHiddens, float max_parameter_range = 1)=0;
    
    virtual void doLearningEpisode(std::vector<std::vector<float> > const& observations, std::vector< std::vector<float> > const& values, float stepSize=0.1, int iterations=1)=0;
    
    virtual std::vector<float> getValues(std::vector<float> const& observations)=0;
    
    virtual void saveApproximator(std::string agentName)=0;
    
    virtual void loadApproximator(std::string agentName)=0;
    
protected:
    std::string save_location;

};

#endif




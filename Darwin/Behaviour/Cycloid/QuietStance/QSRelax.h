/*! @file QSRelax.h
    @brief Declaration of the relaxed state in the QSBallisticController
 
    @class QSRelax
    @brief The relaxed state in the QSBallisticController

    @author Jason Kulk
 
  Copyright (c) 2011 Jason Kulk
 
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

#ifndef QSRelax_H
#define QSRelax_H

#include "Behaviour/BehaviourState.h"
#include "Infrastructure/NUData.h"
class QSBallisticController;

class QSRelax : public BehaviourState
{
public:
    QSRelax(const NUData::id_t& joint, const QSBallisticController* parent);
    ~QSRelax();
    
protected:
    void doState();
    BehaviourState* nextState();
    void updateTargetEstimate();
private:
    NUData::id_t m_joint;
    const QSBallisticController* m_parent;
    double m_time_in_state, m_previous_time;
    
    float m_target_calibration;
};


#endif


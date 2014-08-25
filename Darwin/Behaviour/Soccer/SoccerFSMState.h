/*! @file SoccerState.h
    @brief Declaration of an abstract behaviour state class for other states to inherit from
 
    @class SoccerState
    @brief Declaration of an abstract behaviour state class for other states to inherit from

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

#ifndef SOCCER_FSM_STATE_H
#define SOCCER_FSM_STATE_H

#include "SoccerProvider.h"
#include "Behaviour/BehaviourFSMState.h"

class SoccerFSMState : public BehaviourFSMState
{
public:
    virtual ~SoccerFSMState() {};
    virtual bool stateChanged()
    {
        if (m_state_changed)
            return true;
        if (m_provider and m_provider->stateChanged())
            return true;
        if (m_parent and m_parent->stateChanged())
            return true;
        return false;
    }
protected:
    SoccerFSMState(SoccerProvider* provider) {m_provider = provider; m_parent = 0;};
    SoccerFSMState(SoccerFSMState* parent) {m_parent = parent; m_provider = parent->m_provider;};
    SoccerProvider* m_provider;
    SoccerFSMState* m_parent;
};


#endif


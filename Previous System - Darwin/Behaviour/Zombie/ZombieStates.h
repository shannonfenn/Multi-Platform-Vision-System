/*! @file ChaseBallStates.h
    @brief Chase ball states

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

#ifndef ZOMBIESTATES_H
#define ZOMBIESTATES_H

#include "Behaviour/BehaviourState.h"
#include "ZombieProvider.h"

#include "Infrastructure/Jobs/JobList.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"
#include "Infrastructure/TeamInformation/TeamInformation.h"
#include "Infrastructure/NUBlackboard.h"
#include "Infrastructure/Jobs/MotionJobs/KickJob.h"
#include "Infrastructure/Jobs/MotionJobs/WalkJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadTrackJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadPanJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadNodJob.h"
#include "Infrastructure/Jobs/MotionJobs/MotionFreezeJob.h"

#include "debug.h"

class ZombieSubState : public BehaviourState
{
public:
    ZombieSubState(ZombieProvider* provider){m_provider = provider;};
protected:
    ZombieProvider* m_provider;
};

// ----------------------------------------------------------------------------------------------------------------------- PausedState
class ZombieState : public ZombieSubState
{
public:
    ZombieState(ZombieProvider* provider) : ZombieSubState(provider) {};
    BehaviourState* nextState() {return m_provider->m_state;};
    void doState()
    {
    };
};

#endif

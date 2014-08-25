
// This file is part of Man, a robotic perception, locomotion, and
// team strategy application created by the Northern Bites RoboCup
// team of Bowdoin College in Brunswick, Maine, for the Aldebaran
// Nao robot.
//
// Man is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// Man is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser Public License for more details.
//
// You should have received a copy of the GNU General Public License
// and the GNU Lesser Public License along with Man.  If not, see
// <http://www.gnu.org/licenses/>.

#ifndef _MotionCommand_h_DEFINED
#define _MotionCommand_h_DEFINED

/**
 * Overarching MotionCommand class for motion.
 *
 * It's only ability is to keep track of what kind of motion
 * it implements.
 *
 * Must be DELETED when passed to a provider!!!
 */
#include <list>
#include "MotionConstants.h"

class MotionCommand
{
public:
	MotionCommand(MotionConstants::MotionType type)
		: chainList(), motionType(type) { }
	virtual ~MotionCommand() { }
	const MotionConstants::MotionType getType() const { return motionType; }
	const std::list<int>* getChainList() const { return &chainList; }

protected:
	std::list<int> chainList;
private:
	virtual void setChainList() = 0;
	const MotionConstants::MotionType motionType;
};

#endif

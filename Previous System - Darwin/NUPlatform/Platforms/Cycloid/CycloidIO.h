/*! @file CycloidIO.h
    @brief Declaration of CycloidIO class.
 
    @class CycloidIO
    @brief CycloidIO class for input and output to streams, files and networks on the Bear platform

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

#ifndef CYCLOIDIO_H
#define CYCLOIDIO_H

#include "NUPlatform/NUIO.h"

class NUbot;

class CycloidIO: public NUIO
{
// Functions:
public:
    CycloidIO(NUbot* nubot);
    ~CycloidIO();
    
protected:
private:
};

#endif


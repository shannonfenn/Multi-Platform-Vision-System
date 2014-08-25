/*! 
    @file     ConfigRange.h
    @brief     Header file for the ConfigRange class. 
 
    @class     ConfigRange
    @brief     Templated class used for storing ranges.

    @author Sophie Calland, Mitchell Metcalfe 
 
  Copyright (c) 2012 Sophie Calland, Mitchell Metcalfe 
 
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

#ifndef CONFIG_RANGE_H
#define CONFIG_RANGE_H

#include <boost/foreach.hpp>
#include <vector>

namespace ConfigSystem
{
    //! Enum representing the possible types of boundaries of an interval.
    enum BoundType 
    { 
        bt_none,    //!< none   <==> (-inf, +inf) <==> unrestricted x
        bt_open,    //!< open   <==> (_min, _max) <==> _min <  x >  _max
        bt_closed,  //!< closed <==> [_min, _max] <==> _min <= x >= _max
        bt_unknown  //!< The range has an invalid type.
    };
    
    /*! 
     * This class represents a range, and provides 
     * methods to check whether a value falls within it.
     */
    template <typename T>
    class ConfigRange
    {
        public:
            ConfigRange();

            ConfigRange(T min, 
                        T max, 
                        bool outside  = false, 
                        bool autoClip = false);
            
            ConfigRange(T min,
                        T max,
                        bool outside,
                        bool autoClip,
                        BoundType lBound,
                        BoundType uBound);
            
            ~ConfigRange();

            /*! @brief Retrieves this range's upper boundary value.
                @return Returns this range's upper boundary value.
             */
            T getMax() const;
            
            /*! @brief Retrieves this range's lower boundary value.
                @return Returns this range's lower boundary value
             */
            T getMin() const;
            
            /*! @brief Retrieves the upper bound type of the range.
                @return Returns the BoundType corresponding to the upper bound of the ConfigRange object.
             */
            BoundType getUpperBoundType();
            
            /*! @brief Retrieves the lower bound type of the range.
                @return Returns the BoundType corresponding to the lower bound of the ConfigRange object.
             */
            BoundType getLowerBoundType();


            bool getAutoClip();
            bool getOutside();
            
            //! 'Applies' this range to the given value:
            //!    - clips silently, returning true, if auto-clip is enabled.
            //!    - else, returns whether or not the value fell within the 
            //!      allowed range.
            bool apply(T &value);
            bool apply(std::vector<T> &values);
            bool apply(std::vector< std::vector<T> > &values);
            bool apply(std::vector< std::vector< std::vector<T> > > &values);

            //! Returns whether the given value satisfies the constraints specified
            //! by this range object.
            bool test(T value);
            bool test(std::vector<T> values);
            bool test(std::vector< std::vector<T> > values);
            bool test(std::vector< std::vector< std::vector<T> > > values);
            
            //! Checks whether the given value falls outside this range,
            //! and if it does, minimally adjusts it to satisfy this range
            //! constraint.
            //! Returns whether or not the value was clipped.
            //! Note: bt_open is treated as bt_closed for clipping
            //!       purposes.
            bool clip(T &value);
            //! 'Clips' each value in the given vector (see ConfigRange::clip(T&)).
            bool clip(std::vector<T> &values);
            //! 'Clips' each value in the given vector (see ConfigRange::clip(T&)).
            bool clip(std::vector< std::vector<T> > &values);
            //! 'Clips' each value in the given vector (see ConfigRange::clip(T&)).
			bool clip(std::vector< std::vector< std::vector<T> > > &values);            
            
        private:
            //! Minimum and maximum values in the range
            T _min, _max;

            //! Type of the lower and upper bounds (closed (<=), open (<), or none)
            BoundType _lBound, _uBound;

            //! If true, test() will pass iff the value is not 
            //! between _min and _max.
            //! Ex. _outside = false; ==> [_min, _max]
            //! Ex. _outside = true ; ==> (-inf, _min] U [_max, +inf)
            bool _outside;

            //! If true, indicates that if a value fails to meet this range 
            //! constraint, it should be minimally adjusted such that it does
            //! (i.e. set to the nearest boundary value.
            //! Note: boundaries are currently assumed to be CLOSED when
            //! clipping).
            //! If false, the value should not be changed and failure should
            //! be returned instead.
            bool _autoClip;
    };
}

#include "ConfigRange.template"
#endif


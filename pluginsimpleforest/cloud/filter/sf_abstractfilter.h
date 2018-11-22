/****************************************************************************

    Copyright (C) 2017-2018 , Dr. Jan Hackenberg

    sf_abstractfilter.h is part of SimpleForest - a plugin for the
    Computree platform.

    SimpleForest is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    SimpleForest is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along with
    SimpleForest. If not, see <http://www.gnu.org/licenses/>.

*****************************************************************************/

#ifndef SF_ABSTRACTFILTER_H
#define SF_ABSTRACTFILTER_H

#include "cloud/sf_abstractcloud.h".h"

/**
 * @brief The SF_AbstractFilter class Abstract class for producing segmented or denoised
 * clusters out of a templated PCL cloud
 */
template <typename PointType>
class SF_AbstractFilter: public SF_AbstractCloud
{
public:
    /**
     * @brief SF_AbstractFilter Standard constructor.
     */
    SF_AbstractFilter();
};

#endif // SF_ABSTRACTFILTER_H

template<typename PointType>
SF_AbstractFilter<PointType>::SF_AbstractFilter()
{

}

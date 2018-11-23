/****************************************************************************

    Copyright (C) 2017-2018 , Dr. Jan Hackenberg

    sf_parametersetprincipaldirection.h is part of SimpleForest - a plugin for the
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

#ifndef SF_PARAMETERSETPRINCIPALDIRECTION_H
#define SF_PARAMETERSETPRINCIPALDIRECTION_H

#include "sf_abstractparameterset.h"
#include "sf_parametersetvoxelization.h"

/*! \brief SF_ParameterSetPrincipalDirection.
 *
 *  Parameter set to ccompute the principal direction for a cloud.
 */
template <typename T>
struct SF_ParameterSetPrincipalDirection:
        public SF_AbstractParameterSet<T>
{
    /**
     * @brief m_parameterVoxelization The parameter set storing the \ref m_voxelSize of voxel clustering routine to enable multithreading.
     */
    SF_ParameterSetVoxelization<T> m_parameterVoxelization;

    /**
     * @brief m_normalRadius The radius of the normal computation of \ref  m_cloud.
     */
    float m_normalRadius;
    /**
     * @brief m_pdRadius The radius of the principal direction computation of \ref  m_cloud.
     */
    float m_pdRadius;

    SF_ParameterSetPrincipalDirection() {}
    QStringList paramsToString() override {
        QStringList list;
        QStringList subList = m_parameterVoxelization.paramsToString();
        for(QString subStr : subList)
        {
            list.push_back(std::move(subStr));

        }
        QString str = "The principal direction has been computed with (";
        list.push_back(str);
        str = ("radius normal                = ");
        str.append(QString::number(m_normalRadius));
        str.append("(m)");
        list.push_back(str);
        str = ("radius principal direction   = ");
        str.append(QString::number(m_pdRadius));
        str.append("(m)");
        list.push_back(str);
        str = (" ).");
        list.push_back(str);
        return list;
    }
};

#endif // SF_PARAMETERSETPRINCIPALDIRECTION_H

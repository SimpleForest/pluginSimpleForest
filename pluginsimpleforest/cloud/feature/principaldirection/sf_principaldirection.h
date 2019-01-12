/****************************************************************************

    Copyright (C) 2017-2018 , Dr. Jan Hackenberg

    sf_principaldirection.h is part of SimpleForest - a plugin for the
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

#ifndef SF_PRINCIPALDIRECTION_H
#define SF_PRINCIPALDIRECTION_H

#include "cloud/sf_abstractcloud.h"
#include "parameter/sf_parameterSetPrincipalDirection.h"

/**
 * @brief The SF_PrincipalDirection class. The for the input cloud
 * the principal curvature is computed.
 */
template<typename PointType>
class SF_PrincipalDirection : public SF_AbstractCloudI<PointType>
{
public:
  /**
   * @brief Standard constructor receiving as input \ref m_cloudIn.
   * @param cloudIn \ref m_cloudIn
   */
  SF_PrincipalDirection();
  /**
   * @brief compute Does the computation of the principal curvature of \ref
   * m_cloudIn.
   */
  void compute() override;
  /**
   * @brief setParams Setter for \ref m_params.
   * @param params \ref m_params.
   */
  void setParams(SF_ParameterSetPrincipalDirection<PointType>& params);
  /**
   * @brief principalCurvatures Getter for \ref m_principalCurvatures.
   * @return \ref m_principalCurvatures.
   */
  pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr principalCurvatures();

  /**
   * @brief params Getter for \ref m_params.
   * @return \ref m_params
   */
  SF_ParameterSetPrincipalDirection<PointType> params() const;

private:
  /**
   * @brief m_params The parameter set including normal and principal direction
   * radius.
   */
  SF_ParameterSetPrincipalDirection<PointType> m_params;
  /**
   * @brief m_principalCurvatures A cloud of principal curvatures.
   */
  pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr m_principalCurvatures;
};

#include "sf_principaldirection.hpp"

#endif // SF_PRINCIPALDIRECTION_H

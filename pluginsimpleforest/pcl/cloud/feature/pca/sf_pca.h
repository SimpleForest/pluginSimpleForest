/****************************************************************************

 Copyright (C) 2017-2018 Jan Hackenberg, free software developer
 All rights reserved.

 Contact : https://github.com/SimpleForest

 Developers : Jan Hackenberg

 This file is part of SimpleForest plugin Version 1 for Computree.

 SimpleForest plugin is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 SimpleForest plugin is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with SimpleForest plugin.  If not, see <http://www.gnu.org/licenses/>.

 PluginSimpleForest is an extended version of the SimpleTree platform.

*****************************************************************************/
#ifndef SF_PCA_H
#define SF_PCA_H

#include "pcl/cloud/feature/sf_abstract_feature.h"
#include <pcl/features/normal_3d.h>

struct PCA_Values{
    Eigen::Vector3f lambdas;
    inline void check_valid() {
        if(lambdas[0]>lambdas[1] || lambdas[0]>lambdas[2]||lambdas[1]>lambdas[2]) {
            throw std::runtime_error("Lambdas have not been computed during PCA.");
        }
    }

    inline float getLambda(const int i) {
        if(lambdas[i]!=0) {
            return (lambdas[i]/(lambdas[0]+lambdas[1]+lambdas[2]));
        }
        return 0;
    }

    inline float getLambda1() {
        check_valid();
        return getLambda(0);
    }

    inline float getLambda2() {
        check_valid();
        return getLambda(1);
    }

    inline float getLambda3() {
        check_valid();
        return getLambda(2);
    }

    Eigen::Matrix3f vectors;

    inline Eigen::Vector3f getVector1() {
        check_valid();
        Eigen::Vector3f vec = vectors.col(0);
        vec.normalize();
        return vec;
    }

    inline Eigen::Vector3f getVector2() {
        check_valid();
        Eigen::Vector3f vec = vectors.col(1);
        vec.normalize();
        return vec;
    }



    inline Eigen::Vector3f getVector3() {
        check_valid();
        Eigen::Vector3f vec = vectors.col(2);
        vec.normalize();
        return vec;
    }
};

template <typename PointType>
class SF_PCA: public  SF_AbstractCloud <PointType> {
private:
    float _range;
    bool _use_range;
    int _k;
    bool _center_zero;
    std::vector<PCA_Values> pca_values;
    void extract_neighbors(typename pcl::KdTree<PointType>::Ptr kd_tree, PointType p, typename pcl::PointCloud<PointType>::Ptr neighborhood);
public:
    SF_PCA(typename pcl::PointCloud<PointType>::Ptr cloud_in);
    virtual void compute_features();    
    virtual void createIndices(){}
    virtual void createIndex(PointType point, float sqrd_distance){}
    virtual void reset(){pca_values.clear();}
    static PCA_Values compute_features_from_neighbors(typename pcl::PointCloud<PointType>::Ptr neighborhood, const Eigen::Vector4f& xyz_centroid);
    void compute_features_for_point(const PointType& p, typename pcl::KdTree<PointType>::Ptr kd_tree, int index);
    void set_parameters(float range, bool center_zero, bool use_range );
    void set_parameters(int k, bool center_zero);
    std::vector<PCA_Values> get_pca_values() const;
};


#include "sf_pca.hpp"

#endif // SF_PCA_H


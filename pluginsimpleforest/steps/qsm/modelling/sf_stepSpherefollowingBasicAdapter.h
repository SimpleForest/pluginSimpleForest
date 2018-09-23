#ifndef SF_STEP_SPHEREFOLLOWING_BASIC_ADAPTER_H
#define SF_STEP_SPHEREFOLLOWING_BASIC_ADAPTER_H

#include "steps/param/sf_abstract_param.h"
#include <converters/CT_To_PCL/sf_converterCTToPCL.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>

#include <pcl/ModelCoefficients.h>
#include "qsm/algorithm/sf_buildQSM.h"
#include "qsm/algorithm/sf_QSMAlgorithm.h"
#include "qsm/algorithm/sf_QSMCylinder.h"



class SF_SpherefollowingRootAdapter {
public:

    std::shared_ptr<QMutex>  mMutex;

    SF_SpherefollowingRootAdapter(const SF_SpherefollowingRootAdapter &obj) {
        mMutex = obj.mMutex;
    }

    SF_SpherefollowingRootAdapter () {
        mMutex.reset(new QMutex);
    }

    ~SF_SpherefollowingRootAdapter () {
    }

    void operator()(SF_ParamSpherefollowingBasic<SF_PointNormal> & params) {
        Sf_ConverterCTToPCL<SF_PointNormal> converter;
        {
            QMutexLocker m1(&*mMutex);
            converter.setItemCpyCloudIn(params._itemCpyCloudIn);
        }
        SF_CloudNormal::Ptr cloud;
        SF_CloudNormal::Ptr cloudDownscaled (new SF_CloudNormal());
        converter.compute();
        {
            QMutexLocker m1(&*mMutex);
            params._translation = converter.getCenterOfMass();
            cloud = converter.getCloudTranslated();
        }
        pcl::VoxelGrid<SF_PointNormal> sor;
        sor.setInputCloud (cloud);
        {
            QMutexLocker m1(&*mMutex);
            sor.setLeafSize (params._voxelSize, params._voxelSize, params._voxelSize);
        }
        sor.filter (*cloudDownscaled);
        pcl::NormalEstimation<SF_PointNormal, SF_PointNormal> ne;
        ne.setInputCloud (cloudDownscaled);
        pcl::search::KdTree<SF_PointNormal>::Ptr tree (new pcl::search::KdTree<SF_PointNormal> ());
        ne.setSearchMethod (tree);
        ne.setRadiusSearch (params._voxelSize*3);
        ne.compute (*cloudDownscaled);
        pcl::ModelCoefficients::Ptr coeff1(new pcl::ModelCoefficients);
        coeff1->values.push_back(0);
        coeff1->values.push_back(0);
        coeff1->values.push_back(1);
        coeff1->values.push_back(1);
        pcl::ModelCoefficients::Ptr coeff2(new pcl::ModelCoefficients);
        coeff2->values.push_back(0);
        coeff2->values.push_back(0);
        coeff2->values.push_back(2);
        coeff2->values.push_back(1);
        pcl::ModelCoefficients::Ptr coeff3(new pcl::ModelCoefficients);
        coeff3->values.push_back(0);
        coeff3->values.push_back(0);
        coeff3->values.push_back(3);
        coeff3->values.push_back(1);
        pcl::ModelCoefficients::Ptr coeff4(new pcl::ModelCoefficients);
        coeff4->values.push_back(0);
        coeff4->values.push_back(0);
        coeff4->values.push_back(4);
        coeff4->values.push_back(1);
        pcl::ModelCoefficients::Ptr coeff5(new pcl::ModelCoefficients);
        coeff5->values.push_back(0);
        coeff5->values.push_back(0);
        coeff5->values.push_back(5);
        coeff5->values.push_back(1);
        pcl::ModelCoefficients::Ptr coeff6(new pcl::ModelCoefficients);
        coeff6->values.push_back(0);
        coeff6->values.push_back(0);
        coeff6->values.push_back(6);
        coeff6->values.push_back(1);
        pcl::ModelCoefficients::Ptr coeff7(new pcl::ModelCoefficients);
        coeff7->values.push_back(0);
        coeff7->values.push_back(0);
        coeff7->values.push_back(7);
        coeff7->values.push_back(1);
        pcl::ModelCoefficients::Ptr coeff4a(new pcl::ModelCoefficients);
        coeff4a->values.push_back(0);
        coeff4a->values.push_back(1);
        coeff4a->values.push_back(4);
        coeff4a->values.push_back(1);
        pcl::ModelCoefficients::Ptr coeff4b(new pcl::ModelCoefficients);
        coeff4b->values.push_back(0);
        coeff4b->values.push_back(2);
        coeff4b->values.push_back(4);
        coeff4b->values.push_back(1);
        pcl::ModelCoefficients::Ptr coeff4c(new pcl::ModelCoefficients);
        coeff4c->values.push_back(0);
        coeff4c->values.push_back(3);
        coeff4c->values.push_back(4);
        coeff4c->values.push_back(1);
        pcl::ModelCoefficients::Ptr coeffa4(new pcl::ModelCoefficients);
        coeffa4->values.push_back(1);
        coeffa4->values.push_back(0);
        coeffa4->values.push_back(4);
        coeffa4->values.push_back(1);
        pcl::ModelCoefficients::Ptr coeffb4(new pcl::ModelCoefficients);
        coeffb4->values.push_back(2);
        coeffb4->values.push_back(0);
        coeffb4->values.push_back(4);
        coeffb4->values.push_back(1);
        pcl::ModelCoefficients::Ptr coeffc4(new pcl::ModelCoefficients);
        coeffc4->values.push_back(3);
        coeffc4->values.push_back(0);
        coeffc4->values.push_back(4);
        coeffc4->values.push_back(1);

        SF_QSMDetectionCylinder cyl1(0,coeff1, coeff2);
        SF_QSMDetectionCylinder cyl2(0,coeff2, coeff3);
        SF_QSMDetectionCylinder cyl3(0,coeff3, coeff4);
        SF_QSMDetectionCylinder cyl4(0,coeff4, coeff4a);
        SF_QSMDetectionCylinder cyl5(0,coeff4a, coeff4b);
        SF_QSMDetectionCylinder cyl6(0,coeff4b, coeff4c);
        SF_QSMDetectionCylinder cyl7(0,coeff4, coeff5);
        SF_QSMDetectionCylinder cyl8(0,coeff5, coeff6);
        SF_QSMDetectionCylinder cyl9(0,coeff6, coeff7);
        SF_QSMDetectionCylinder cyl10(0,coeff4, coeffa4);
        SF_QSMDetectionCylinder cyl11(0,coeffa4, coeffb4);
        SF_QSMDetectionCylinder cyl12(0,coeffb4, coeffc4);

        std::vector<SF_QSMDetectionCylinder> list;
        list.push_back(cyl1);
        list.push_back(cyl2);
        list.push_back(cyl3);
        list.push_back(cyl4);
        list.push_back(cyl5);
        list.push_back(cyl6);
        list.push_back(cyl7);
        list.push_back(cyl8);
        list.push_back(cyl9);
        list.push_back(cyl10);
        list.push_back(cyl11);
        list.push_back(cyl12);
        {
            QMutexLocker m1(&*mMutex);
            SF_BuildQSM builder(list, 666);
            std::shared_ptr<SF_ModelQSM> tree2 =  builder.getTree();
            std::vector< std::shared_ptr<Sf_ModelAbstractBuildingbrick> > bricks = tree2->getBuildingBricks();
            for(size_t i = 0; i < bricks.size(); i++) {
                std::shared_ptr<Sf_ModelAbstractBuildingbrick> brick = bricks[i];
            }
        }






//        params.log_import();
//        SF_Statistical_Outlier_Filter<SF_Point> filter;
//        {
//            QMutexLocker m1(&*mMutex);
//            filter.set_cloud_in(params._cloud_in);
//        }
//        filter.compute(params);
//        {
//            QMutexLocker m1(&*mMutex);
//            params._output_indices = filter.get_indices();
//        }
//        params.log_filter(filter.get_percentage());
    }
};

#endif // SF_STEP_SPHEREFOLLOWING_BASIC_ADAPTER_H

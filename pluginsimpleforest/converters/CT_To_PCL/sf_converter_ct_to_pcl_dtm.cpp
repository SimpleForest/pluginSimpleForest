#include "sf_converter_ct_to_pcl_dtm.h"

SF_Converter_CT_to_PCL_DTM::SF_Converter_CT_to_PCL_DTM(Eigen::Vector3d centerOfMass, CT_Image2D<float> *dtmCT) {
    _dtmCT = dtmCT;
    _centerOfMass = centerOfMass;
    compute();
}

SF_Converter_CT_to_PCL_DTM::SF_Converter_CT_to_PCL_DTM(CT_Image2D<float> * dtmCT) {
    _dtmCT = dtmCT;
    computeTranslationToOrigin();
    compute();
}

std::shared_ptr<SF_DTM_Model> SF_Converter_CT_to_PCL_DTM::dtmPCL() const {
    return _dtmPCL;
}

void SF_Converter_CT_to_PCL_DTM::computeTranslationToOrigin() {
    Eigen::Vector2d min, max;
    _dtmCT->getMinCoordinates(min);
    _dtmCT->getMaxCoordinates(max);
    _centerOfMass[0] = (min[0] + max[0])/2;
    _centerOfMass[1] = (min[1] + max[1])/2;
    size_t index;
    _dtmCT->index(_dtmCT->xArraySize()/2, _dtmCT->yArraySize()/2, index);
    _centerOfMass[2] = _dtmCT->valueAtIndex(index);
}

void SF_Converter_CT_to_PCL_DTM::compute() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for(size_t i = 0; i < _dtmCT->nCells(); i++) {
        Eigen::Vector2d bottom,top;
        _dtmCT->getCellCoordinates(i, bottom, top);
        pcl::PointXYZ point;
        point.x = (bottom[0]+top[0])/2    - _centerOfMass[0];
        point.y = (bottom[1]+top[1])/2    - _centerOfMass[1];
        point.z = _dtmCT->valueAtIndex(i) - _centerOfMass[2];
        cloud->points.push_back(point);
    }
    _dtmPCL.reset(new SF_DTM_Model(cloud));
}

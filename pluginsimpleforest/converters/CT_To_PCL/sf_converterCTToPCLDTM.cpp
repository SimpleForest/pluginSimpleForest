#include "sf_converterCTToPCLDTM.h"

SF_ConverterCTToPCLDTM::SF_ConverterCTToPCLDTM(Eigen::Vector3d centerOfMass, CT_Image2D<float> *dtmCT) {
    _dtmCT = dtmCT;
    m_translation = centerOfMass;
    compute();
}

SF_ConverterCTToPCLDTM::SF_ConverterCTToPCLDTM(CT_Image2D<float> * dtmCT) {
    _dtmCT = dtmCT;
    computeTranslationToOrigin();
    compute();
}

std::shared_ptr<SF_ModelDTM> SF_ConverterCTToPCLDTM::dtmPCL() const {
    return _dtmPCL;
}

void SF_ConverterCTToPCLDTM::computeTranslationToOrigin() {
    Eigen::Vector2d min, max;
    _dtmCT->getMinCoordinates(min);
    _dtmCT->getMaxCoordinates(max);
    m_translation[0] = (min[0] + max[0])/2;
    m_translation[1] = (min[1] + max[1])/2;
    size_t index;
    _dtmCT->index(_dtmCT->xArraySize()/2, _dtmCT->yArraySize()/2, index);
    m_translation[2] = _dtmCT->valueAtIndex(index);
}

void SF_ConverterCTToPCLDTM::compute() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for(size_t i = 0; i < _dtmCT->nCells(); i++) {
        Eigen::Vector2d bottom,top;
        _dtmCT->getCellCoordinates(i, bottom, top);
        pcl::PointXYZ point;
        point.x = (bottom[0]+top[0])/2    - m_translation[0];
        point.y = (bottom[1]+top[1])/2    - m_translation[1];
        point.z = _dtmCT->valueAtIndex(i) - m_translation[2];
        cloud->points.push_back(point);
    }
    _dtmPCL.reset(new SF_ModelDTM(cloud));
}

#ifndef SF_CONVERTER_CT_TO_PCL_DTM_H
#define SF_CONVERTER_CT_TO_PCL_DTM_H

#include "converters/sf_abstract_converter.h"
#include "plot/DTM/sf_dtm.h"
#include "ct_itemdrawable/ct_image2d.h"

class SF_Converter_CT_to_PCL_DTM: public SF_Abstract_Converter {
    std::shared_ptr<SF_DTM_Model> _dtmPCL;
    CT_Image2D<float> * _dtmCT;
    virtual void computeTranslationToOrigin();
    virtual void reset(){}
    virtual void compute();
public:
    SF_Converter_CT_to_PCL_DTM(CT_Image2D<float> * dtmCT);
    SF_Converter_CT_to_PCL_DTM(Eigen::Vector3d centerOfMass, CT_Image2D<float> * dtmCT);
    std::shared_ptr<SF_DTM_Model> dtmPCL() const;
};

#endif // SF_CONVERTER_CT_TO_PCL_DTM_H

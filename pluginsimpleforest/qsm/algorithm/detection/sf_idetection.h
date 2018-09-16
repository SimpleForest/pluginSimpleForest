#ifndef SF_IDETECTION_H
#define SF_IDETECTION_H

#include "qsm/sf_model_tree.h"

class SF_IDetection {
public:
    virtual std::shared_ptr<SF_Model_Tree> getQSM() = 0;
    virtual void compute() = 0;
    virtual void error() = 0;
};


#endif // SF_IDETECTION_H

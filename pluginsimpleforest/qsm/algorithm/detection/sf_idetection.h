#ifndef SF_IDETECTION_H
#define SF_IDETECTION_H

#include "qsm/sf_modelQSM.h"

class SF_IDetection {
  std::shared_ptr<SF_ModelQSM> _qsm;

public:
  const virtual std::shared_ptr<SF_ModelQSM> getQSM() = 0;
  virtual void compute() = 0;
  virtual float error() = 0;
};

#endif // SF_IDETECTION_H

#ifndef SEM_ENCODING_DISTRIBUTIONSAMPLER1D_H_
#define SEM_ENCODING_DISTRIBUTIONSAMPLER1D_H_

#include "encoding/base_distributionsampler.h"

class DistributionSampler1D : public base_DistributionSampler
{
public:
    DistributionSampler1D();

    int Sample() const;
};

#endif // SEM_ENCODING_DISTRIBUTIONSAMPLER1D_H_

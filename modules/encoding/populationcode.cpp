#include "encoding/populationcode.h"

using cv::Mat1f;

base_PopulationCode::base_PopulationCode()
{
}

MutexPopulationCode::MutexPopulationCode()
    : base_PopulationCode()
{
}

void MutexPopulationCode::State(const Mat1f& in, const VecMat1f& kernels)
{
    state_ = Mat1f::zeros(in.rows*2, 1);
    for(int i=0, j=0; i < in.rows; i++, j+=2) {

        int k = (in(i) < 0.5f) ? 0 : 1;
        state_(j+k) = 1.f;
    }
}

Mat1f MutexPopulationCode::PopCode()
{
    return state_;
}

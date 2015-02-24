/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/encoding/populationcode_derivs/mutex_populationcode.h"

using std::string;

using cv::Mat1f;

using namespace elm;

MutexPopulationCode::MutexPopulationCode()
    : base_PopulationCode()
{
    Clear();
}

MutexPopulationCode::MutexPopulationCode(const LayerConfig &config)
    : base_PopulationCode(config)
{
    Reset(config);
}

void MutexPopulationCode::State(const Mat1f& in, const VecMat1f& kernels)
{
    pop_code_ = Mat1f::zeros(in.rows, in.cols*2);
    for(int i=0, j=0; i < static_cast<int>(in.total()); i++, j+=2) {

        int k = (in(i) < 0.5f) ? 0 : 1;
        pop_code_(j+k) = 1.f;
    }
}

Mat1f MutexPopulationCode::PopCode()
{
    return pop_code_;
}


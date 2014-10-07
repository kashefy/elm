#include "encoding/populationcode.h"

base_PopulationCode::base_PopulationCode()
{
}

MutexPopulationCode::MutexPopulationCode()
    : base_PopulationCode()
{
}

void MutexPopulationCode::State(const MatF& in, const VecMatF& kernels)
{
    state_ = MatF::zeros(in.rows*2, 1);
    for(int i=0, j=0; i < in.rows; i++, j+=2) {

        int k = (in(i) < 0.5f) ? 0 : 1;
        state_(j+k) = 1.f;
    }
}

MatF MutexPopulationCode::PopCode()
{
    return state_;
}

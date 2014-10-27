#include "encoding/ganglion.h"

using namespace cv;

base_Ganglion::~base_Ganglion()
{
}

base_Ganglion::base_Ganglion()
{
}

DiffOfGaussians2dSq::DiffOfGaussians2dSq()
    : base_Ganglion()
{
}

void DiffOfGaussians2dSq::Init(int radius, float sigma_c, float sigma_s, bool center_on)
{
}

Mat1f DiffOfGaussians2dSq::Kernel() const
{
    return Mat1f();
}

Mat1f DiffOfGaussians2dSq::Compute()
{
    return Mat1f();
}

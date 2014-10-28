#include "encoding/ganglion.h"

#include <opencv2/imgproc.hpp>

#include "core/exception.h"

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
#include <iostream>
void DiffOfGaussians2dSq::Init(int radius, float scale, bool center_on)
{
    //sigma = 0.3*((ksize-1)*0.5 - 1) + 0.8
    float sigma_center = scale*2;
    float sigma_surround = scale*10;

    if(radius < 1) {

        SEM_THROW_BAD_DIMS("Diff. of Gaussians radius must be > 0.");
    }

    int size = radius*2+1;
    Mat center = getGaussianKernel(size, sigma_center, CV_32F);
    Mat surround = getGaussianKernel(size, sigma_surround, CV_32F);

    kernel_1d_ = (center_on)? center-surround : surround-center;
}

Mat1f DiffOfGaussians2dSq::Kernel() const
{
    int size = static_cast<int>(kernel_1d_.total());

    Mat1f impulse = Mat1f::zeros(size, size);
    impulse(size/2, size/2) = 1.f;

    return const_cast<DiffOfGaussians2dSq*>(this)->Compute(impulse);
}

Mat1f DiffOfGaussians2dSq::Compute(InputArray in)
{
    Mat1f out;
    sepFilter2D(in,
                out, -1,
                kernel_1d_, kernel_1d_,
                Point(-1,-1), 0, BORDER_REPLICATE);
    return out;
}

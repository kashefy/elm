#include "sem/encoding/ganglion.h"

#include <opencv2/imgproc.hpp>

#include "sem/core/exception.h"

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
    sigma_center_ = scale*2;
    sigma_surround_ = scale*10;

    if(radius < 1) {

        SEM_THROW_BAD_DIMS("Diff. of Gaussians radius must be > 0.");
    }

    size_ = radius*2+1;
//    Mat center = getGaussianKernel(size_, sigma_center_, CV_32F);
//    Mat surround = getGaussianKernel(size_, sigma_surround_, CV_32F);

    is_center_on_ = center_on;
}

Mat1f DiffOfGaussians2dSq::Kernel() const
{
    Mat1f impulse = Mat1f::zeros(size_, size_);
    impulse(size_/2, size_/2) = 1.f;

    return const_cast<DiffOfGaussians2dSq*>(this)->Compute(impulse);
}

Mat1f DiffOfGaussians2dSq::Compute(InputArray in)
{
    Mat r_surround, r_center, r_diff;
    GaussianBlur(in, r_center,
                 Size2i(size_, size_),
                 sigma_center_, sigma_center_,
                 BORDER_REPLICATE);

    GaussianBlur(in, r_surround,
                 Size2i(size_, size_),
                 sigma_surround_, sigma_surround_,
                 BORDER_REPLICATE);

    r_diff = (is_center_on_)? r_center-r_surround : r_surround-r_center;

    return r_diff;
}

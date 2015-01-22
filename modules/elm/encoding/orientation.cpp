#include "elm/encoding/orientation.h"

#include <opencv2/imgproc.hpp>

#include "elm/core/defs.h"
#include "elm/core/exception.h"

using namespace cv;
using namespace elm;

GaborFilterBank::GaborFilterBank()
    : base_FilterBank()
{
}

int GaborFilterBank::Reset(int radius,
                           float sigma,
                           const VecF &theta_rad,
                           float lambd,
                           float gamma,
                           float ps)
{
    kernels_ = CreateKernels(radius,
                             sigma,
                             theta_rad,
                             lambd,
                             gamma,
                             ps);
    return static_cast<int>(kernels_.size());
}

Mat1f GaborFilterBank::CreateKernel(int radius,
                  float sigma,
                  float theta_rad,
                  float lambd,
                  float gamma,
                  float ps)
{
    theta_rad = -theta_rad + ELM_PI2;
    if(radius < 1) { ELM_THROW_VALUE_ERROR("Gabor kernel radius must be >= 1."); }
    int diam = radius*2+1;
    Mat1f kernel = getGaborKernel(Size2i(diam, diam),
                                  sigma, theta_rad, lambd, gamma,
                                  ps, CV_32F);
    kernel /= sum(kernel)(0);
    return kernel;
}

VecMat1f GaborFilterBank::CreateKernels(int radius,
                                        float sigma,
                                        const VecF &theta_rad,
                                        float lambd,
                                        float gamma,
                                        float ps)
{
    int N = static_cast<int>(theta_rad.size());
    VecMat1f filter_bank;
    filter_bank.reserve(N);

    for(int i=0; i<N; i++) {

        Mat1f kernel = CreateKernel(radius, sigma, theta_rad[i], lambd, gamma, ps);
        filter_bank.push_back(kernel);
    }
    return filter_bank;
}

VecMat1f GaborFilterBank::Kernels() const
{
    return kernels_;
}

void GaborFilterBank::Rectify(Mat1f &response)
{
    pow(response, 2., response);
}

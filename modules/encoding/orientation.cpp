#include "encoding/orientation.h"

#include <opencv2/imgproc.hpp>

#include "core/defs.h"
#include "core/exception.h"

using namespace cv;
using namespace sem;

Mat1f sem::GaborKernel(int radius, float sigma, float theta_rad, float lambd, float gamma, float ps)
{
    theta_rad = -theta_rad + SEM_PI2;
    if(radius < 1) { SEM_THROW_VALUE_ERROR("Gabor kernel radius must be >= 1."); }
    int diam = radius*2+1;
    Mat1f kernel = getGaborKernel(Size2i(diam, diam),
                                  sigma, theta_rad, lambd, gamma,
                                  ps, CV_32F);
    kernel /= sum(kernel)(0);
    return kernel;
}

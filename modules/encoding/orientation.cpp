#include "encoding/orientation.h"

#include <opencv2/imgproc.hpp>

using namespace cv;
using namespace sem;

Mat1f sem::GaborKernel(int radius, float sigma, float theta, float lambd, float gamma, float ps)
{
    int diam = radius*2+1;
    Mat1f kernel = getGaborKernel(Size2i(diam, diam),
                                  sigma, theta, lambd, gamma,
                                  ps, CV_32F);

    return kernel;
}

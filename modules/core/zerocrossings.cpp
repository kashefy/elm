#include "core/zerocrossings.h"

#include <opencv2/imgproc.hpp>

using namespace cv;

base_ZeroCrossings::~base_ZeroCrossings()
{
}

base_ZeroCrossings::base_ZeroCrossings()
{
}

ZeroCrossingsSobel::ZeroCrossingsSobel()
    : base_ZeroCrossings()
{
}

void ZeroCrossingsSobel::operator ()(InputArray src, OutputArray dst) const
{
    Mat sx, sy;
    Sobel(src, sx, CV_32F, 1, 0, 1);
    Sobel(src, sy, CV_32F, 0, 1, 1);

    // Compute Laplacian
    Mat laplace;
    Laplacian(src, laplace, CV_32F, 3);

    // Create the iterators
    Mat1fCIter it    = laplace.begin<float>() + laplace.step1();
    Mat1fCIter itend = laplace.end<float>();
    Mat1fCIter itup  = laplace.begin<float>();
    Mat1fCIter itx   = sx.begin<float>() + sx.step1();
    Mat1fCIter ity   = sy.begin<float>() + sy.step1();

    // Binary image initialize to white
    float threshold = 0.f;
    Mat binary(laplace.size(), CV_8U, Scalar(255));
    Mat_<uchar>::iterator itout = binary.begin<uchar>() + binary.step1();

    for ( ; it != itend; ++it, ++itup, ++itout, ++itx, ++ity) {

      // if the product of two adjacent pixel is negative
      // then there is a sign change
      if (*it * *(it-1) < 0.f && fabs(static_cast<double>(*ity)) > threshold) {

          *itout = 0; // horizontal zero-crossing
      }
      else if (*it * *itup < 0.f && fabs(*ity) > threshold) {

          *itout = 0; // vertical zero-crossing
      }
    }

    binary.copyTo(dst);
}

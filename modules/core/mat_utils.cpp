#include "core/mat_utils.h"

#include <iostream>

using namespace std;
using namespace cv;

Mat_<uchar> sem::ConvertTo8U(const Mat &src)
{
    double min_val, max_val;
    int max_idx[2] = {-1, -1};
    minMaxIdx(src, &min_val, 0, 0, max_idx);

    cv::Mat_<double> tmp = src-min_val;

    max_val = tmp(max_idx[0], max_idx[1]);
    if(max_val != 0) {
        tmp /= max_val;
    }

    Mat_<uchar> dst;
    tmp.convertTo(dst, CV_MAKETYPE(CV_8U, src.channels()), 255.);
    return dst;
}

void sem::CumSum(const Mat1f &src, Mat1f &dst)
{
    if(dst.total() < src.total() && src.total() > 0) {

        dst = Mat1f::zeros(src.rows, src.cols);
    }

    float interm_sum = 0;
    for(unsigned int i = 0; i < src.total(); i++) {

        interm_sum += src(i);
        dst(i) = interm_sum;
    }
}

string sem::MatTypeToString(const cv::Mat& m)
{
    std::string type_name;
    uchar depth = m.type() & CV_MAT_DEPTH_MASK;

    switch ( depth ) {
      case CV_8U:   type_name = "CV_8U"  ; break;
      case CV_8S:   type_name = "CV_8S"  ; break;
      case CV_16U:  type_name = "CV_16U" ; break;
      case CV_16S:  type_name = "CV_16S" ; break;
      case CV_32S:  type_name = "CV_32S" ; break;
      case CV_32F:  type_name = "CV_32F" ; break;
      case CV_64F:  type_name = "CV_64F" ; break;
      default: break;
    }
    return type_name;
}

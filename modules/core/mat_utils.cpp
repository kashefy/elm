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

Mat1i sem::Point2Mat(const Point2i &p)
{
    Mat1i m(1, 2);
    m(0) = p.x;
    m(1) = p.y;
    return m;
}

void sem::NeighMeanVar(const Mat1f &m, int radius, Mat1f &neigh_mean, Mat1f &neigh_var)
{
    neigh_mean = Mat1f(m.size());
    neigh_var = Mat1f(m.size());

    for(int r=0; r<m.rows; r++) {

        Mat1f sub_rows = m.rowRange(r < radius? 0 : r,
                                    r+radius+1 > m.rows? m.rows : r);

        for(int c=0; c<m.cols; c++) {

            Mat1f sub_mat = sub_rows.rowRange(c < radius? 0 : c,
                                               c+radius+1 > m.cols? m.cols : c);

            Mat sub_mean, sub_stddev;
            meanStdDev(sub_mat, sub_mean, sub_stddev);
            neigh_mean(r, c) = static_cast<float>(sub_mean.at<double>(0));
            neigh_var(r, c) = static_cast<float>(sub_stddev.at<double>(0)); // don't forget to square
        }
    }
    multiply(neigh_var, neigh_var, neigh_var);
}

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

void sem::NeighMeanVar(const Mat1f &m, int radius, Mat1f &neigh_mean, Mat1f &neigh_var,
                       int border_type, const Scalar &value)
{
    int diameter = radius*2+1;

    // check radius
    if(radius < 0) {
        SEM_THROW_VALUE_ERROR("radius must be >= 0");
    }
    else if(radius == 0) {

        // we can short-circuit the resuls for this case
        neigh_mean = m.clone();
        neigh_var = Mat1f::zeros(m.size());
    }
    else {
        neigh_mean = Mat1f(m.size());
        neigh_var = Mat1f(m.size());

        // pad borders
        Mat1f padded;
        copyMakeBorder(m, padded,
                       radius, radius, radius, radius,
                       border_type, value);

        for(int r=0; r<m.rows; r++) {

            Mat1f sub_rows = padded.rowRange(r, r+diameter);

            for(int c=0; c<m.cols; c++) {

                Mat sub_mean, sub_stddev;
                meanStdDev(sub_rows.colRange(c, c+diameter),
                           sub_mean, sub_stddev);

                neigh_mean(r, c) = static_cast<float>(sub_mean.at<double>(0));
                neigh_var(r, c) = static_cast<float>(sub_stddev.at<double>(0)); // don't forget to square
            }
        }
        multiply(neigh_var, neigh_var, neigh_var);
    }
}

Mat1f sem::ElementsAt(const VecMat1f &v, int r, int c)
{
    Mat1f values(1, static_cast<int>(v.size()));
    int k=0;
    for(VecMat1fCIter itr=v.begin();
        itr != v.end();
        itr++, k++) {

        if(r < 0 || r >= (*itr).rows || c< 0 || c >= (*itr).cols) {

            stringstream s;
            s << "Cannot access element at (" << r << "," << c << ") in v[" << k << "]"
              << " with dims (" << (*itr).rows << "," << (*itr).cols <<")";
            SEM_THROW_BAD_DIMS(s.str());
        }
        else {
            values(k) = (*itr)(r, c);
        }
    }

    return values;
}

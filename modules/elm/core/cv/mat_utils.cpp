/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/core/cv/mat_utils.h"

#include "elm/core/exception.h"

using namespace std;
using namespace cv;

Mat_<uchar> elm::ConvertTo8U(const Mat &src)
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

void elm::CumSum(const Mat1f &src, Mat1f &dst)
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

int elm::tril_flat(const Mat1f &src, Mat1f &dst)
{
    int nb_ab = 0;
    if(!src.empty()) {

        for(int i=src.rows; i>0; i--) {
            nb_ab += i;
        }

        dst = Mat1f(1, nb_ab);

        int i=0;
        for(int r=0; r<src.rows; r++) {
            for(int c=0; c<=r; c++) {

                dst(i++) = src(r, c);
            }
        }
    }

    return nb_ab;
}

Mat1f elm::diff(const Mat1f &src, int n, int axis)
{
    Mat1f d(src.size());

    if(!src.empty()) {

    }

    ELM_THROW_NOT_IMPLEMENTED;

    return d;
}

Mat1i elm::Point2Mat(const Point2i &p)
{
    Mat1i m(1, 2);
    m(0) = p.x;
    m(1) = p.y;
    return m;
}

Point2i elm::Mat2Point2i(const Mat1i &m)
{
    if(m.total() < 2) {
        ELM_THROW_BAD_DIMS("Too few matrix elements for extracting x,y coordinates. Must have at least 2 elements");
    }
    return Point2i(m(0), m(1));
}

namespace elm
{
template<>
Point3i Mat2Point3_(const Mat1i &m)
{
    if(m.total() < 3) {
        ELM_THROW_BAD_DIMS("Too few matrix elements for extracting x,y,z coordinates. Must have at least 3 elements");
    }
    return Point3i(m(0), m(1), m(2));
}

template<>
Point3f Mat2Point3_(const Mat1f &m)
{
    if(m.total() < 3) {
        ELM_THROW_BAD_DIMS("Too few matrix elements for extracting x,y,z coordinates. Must have at least 3 elements");
    }
    return Point3f(m(0), m(1), m(2));
}

Mat1b isnan(const Mat1f &src)
{
    if(src.empty()) {

        return Mat1b();
    }

    return (src != src);
}

Mat1b is_not_nan(const Mat1f &src)
{
    if(src.empty()) {

        return Mat1b();
    }

    return (src == src);
}

} //namespace elm


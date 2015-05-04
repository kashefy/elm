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
    Mat1f d;

    if(!src.empty()) {

        if(axis < 0) {

            // use last axis
            axis = src.dims-1;
        }
        else if(axis >= src.dims) {

            stringstream s;
            s << "Invalid axis index (" << axis << ") for "
              << src.dims << "-dimensional Mat";
            ELM_THROW_VALUE_ERROR(s.str());
        }

        if(src.dims > 2) {

            ELM_THROW_NOT_IMPLEMENTED_WMSG("Diff on Mat with dims > 2 not yet supported.");
        }

        if(n==0) {

            src.copyTo(d); // return data as is
        }
        else if(n==1) {

            if(axis == 0) {

                // across rows
                d = src.rowRange(1, src.rows) - src.rowRange(0, src.rows-1);
            }
            else if(axis == 1) {

                // across columns
                d = src.colRange(1, src.cols) - src.colRange(0, src.cols-1);
            }
        }
        else {

            ELM_THROW_NOT_IMPLEMENTED_WMSG("higher order derivatives not yet supported.");
        }
    }

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

Mat1i Rect2iToMat(const Rect2i &r)
{
    Mat1i m(1, 4);
    Point2Mat(r.tl()).copyTo(m.colRange(0, 2));
    Point2Mat(r.br()).copyTo(m.colRange(2, 4));
    return m;
}

Rect2i MatToRect2i(const Mat1i &m)
{
    ELM_THROW_BAD_DIMS_IF(m.total() < 4, "Mat must have at least 4 elements to convert to Rect");

    int x = m(0);
    int y = m(1);
    return Rect2i(x, y, m(2)-x, m(3)-y);
}

Mat1b isnan(const Mat1f &src)
{
    if(src.empty()) {

        return Mat1b();
    }

    Mat1b dst;
    compare(src, src, dst, CMP_NE);

    return dst;
}

Mat1b is_not_nan(const Mat1f &src)
{
    if(src.empty()) {

        return Mat1b();
    }

    Mat1b dst;
    compare(src, src, dst, CMP_EQ);

    return dst;
}

} //namespace elm


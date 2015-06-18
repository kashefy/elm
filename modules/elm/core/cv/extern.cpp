/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/core/cv/typedefs_fwd.h"

#include <opencv2/core/core.hpp>

template class cv::Mat_<float>;
template class cv::Mat_<int>;
template class cv::Mat_<uchar>;

template class std::vector<cv::Mat>;
template class std::vector<cv::Mat_<float> >;

template class cv::SparseMat_<float>;

template class cv::Rect_<int>;
template class cv::Size_<int>;
template class cv::Point_<int>;
template class cv::Point3_<int>;

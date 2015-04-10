/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_CORE_BOOST_SERIALIZATION_SER_MAT_UTILS_H_
#define _ELM_CORE_BOOST_SERIALIZATION_SER_MAT_UTILS_H_

#include "elm/core/boost/serialization/ser_mat.h"

namespace elm {

template<class Tarchive, class TElem>
void Save(Tarchive &ar, const cv::Mat_<TElem> &obj)
{
    cv::Mat m = static_cast<cv::Mat>(obj);
    ar & m;
}

template<class Tarchive, class TElem>
void Load(Tarchive &ar, cv::Mat_<TElem> &obj)
{
    cv::Mat m;
    ar & m;
    obj = static_cast<cv::Mat_<TElem> >(m);
}

} // namespace elm

#endif // _ELM_CORE_BOOST_SERIALIZATION_SER_MAT_UTILS_H_

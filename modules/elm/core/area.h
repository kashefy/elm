/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_CORE_AREA_H_
#define _ELM_CORE_AREA_H_

#include "elm/core/cv/typedefs_fwd.h"

class base_AUC
{
public:
    virtual ~base_AUC() {}

    virtual float operator ()(const cv::Mat1f &x, const cv::Mat1f &y) const = 0;

protected:
    base_AUC();
};

class Trapz : public base_AUC
{
public:
    Trapz();

    virtual float operator ()(const cv::Mat1f &x, const cv::Mat1f &y) const;

protected:
    float Trapezoidal(float x1, float x2, float y1, float y2) const;
};

#endif // _ELM_CORE_AREA_H_

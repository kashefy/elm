/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, Elm Library Project
// 3-clause BSD License
//
//M*/
#include "elm/core/area.h"

#include <opencv2/core.hpp>

#include "elm/core/exception.h"

using namespace cv;

base_AUC::base_AUC()
{
}

Trapz::Trapz()
    : base_AUC()
{
}

float Trapz::operator ()(const Mat1f &x, const Mat1f &y) const
{
    if(x.total() < 2 || y .total() < 2) {

        ELM_THROW_BAD_DIMS("x and y dims must both have >= 2 elements.");
    }

    float area = 0.f;
    for(int i=1; i<static_cast<int>(x.total()); i++) {

        area += Trapezoidal(x(i-1), x(i), y(i-1), y(i));
    }
    return area;
}

float Trapz::Trapezoidal(float x1, float x2, float y1, float y2) const
{
    float h = x2-x1;
    // Evaluate endpoints
    float area = h*(y1+y2)/2.f;
    return area;
}


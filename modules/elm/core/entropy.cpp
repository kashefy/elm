/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/core/entropy.h"

#include "elm/core/exception.h"

using namespace cv;

float elm::CondEntropy(InputArray pdf)
{
    if(countNonZero(pdf) < static_cast<int>(pdf.total())) {
        ELM_THROW_VALUE_ERROR("Failed to compute cond. entropy with zero in prob. vector");
    }

    if(pdf.total() == 0) { return 0.f; }

    Mat1f prod, lg;
    log(pdf, lg);
    multiply(pdf, lg, prod);
    return -sum(prod)(0);
}

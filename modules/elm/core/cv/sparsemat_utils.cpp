/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/core/cv/sparsemat_utils.h"

#include <opencv2/core/core.hpp>

size_t elm::total(const cv::SparseMat &m)
{
    const int D = m.dims();

    int n = (D>0)? 1 : 0;

    for(int i=0; i<D; i++) {

        n *= m.size(i);
    }
    return static_cast<size_t>(n);
}

/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/io/matio_utils.h"

#include <opencv2/core/core.hpp>

#include "elm/core/exception.h"

using namespace cv;

#ifdef __WITH_MATIO

unsigned int elm::MATIOClassTOCV_TYPE(matio_classes type)
{
    unsigned int cv_type;
    switch(type) {

    case MAT_C_DOUBLE:
        cv_type = CV_64F;
        break;
    case MAT_C_UINT16:
        cv_type = CV_16U;
        break;

    default:
        ELM_THROW_TYPE_ERROR("unrecognized type");
    }

    return cv_type;
}

#endif // __WITH_MATIO

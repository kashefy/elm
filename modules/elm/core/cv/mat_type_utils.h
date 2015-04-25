/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_CORE_CV_MAT_TYPE_UTILS_H_
#define _ELM_CORE_CV_MAT_TYPE_UTILS_H_

#include <string>

#include "elm/core/cv/typedefs_fwd.h"

namespace elm {

/**
 * @brief Get string representation of matrix type
 * @param matrix
 * @return type as string
 */
std::string MatTypeToString(const cv::Mat& m);

} // namespace elm

#endif // _ELM_CORE_CV_MAT_TYPE_UTILS_H_

/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_CORE_CV_SPARSEMAT_UTILS_H_
#define _ELM_CORE_CV_SPARSEMAT_UTILS_H_

#include "elm/core/typedefs_fwd.h"

namespace elm
{

/**
 * @brief Get total number of elements that can be occupied in this sparse matrix
 * @param m sparse mat
 * @return product of sizes
 */
size_t total(const cv::SparseMat &m);

} // namespace elm

#endif // _ELM_CORE_CV_SPARSEMAT_UTILS_H_

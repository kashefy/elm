/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, Elm Library Project
// 3-clause BSD License
//
//M*/
/** @file template utility definitions using STL vectors that have to be defined inline
  */
#ifndef _ELM_CORE_MAT_VECTOR_UTILS_INL_H_
#define _ELM_CORE_MAT_VECTOR_UTILS_INL_H_

#include <vector>

#include "elm/core/cv/typedefs_fwd.h"
#include "elm/core/exception.h"

namespace elm {

/**
 * @brief Function for converting a Mat_ object into a vector of same type
 * The mat is flattened beforehand.
 * Involves copying elements
 * May only work with matrices of POD (e.g. int, float, uchar,...)
 * Works with N-dimensional matrices
 *
 * @param matrix
 * @return resulting vector with flattened matrix data
 * @throws ExceptionTypeError on non-continuous matrix input
 */
template <typename T>
std::vector<T> Mat_ToVec_(const cv::Mat_<T> &m) {

    if(!m.empty() && !m.isContinuous()) {

        ELM_THROW_TYPE_ERROR("Only conitnuous matrices supported.");
    }
    // syntax learned from posts here:
    // http://stackoverflow.com/questions/610245/where-and-why-do-i-have-to-put-the-template-and-typename-keywords
    const T* p = m.template ptr<T>(0);
    std::vector<T> v(p, p+m.total());
    return v;
}

/** @brief Function for converting STL vector of POD type into an OpenCV Mat.
 *
 * No deep copy.
 * The data is not copied to the matrix.
 * The data remains owned by the vector
 * The resulting mat object should not be referenced if the source vector goes out of scope
 *
 * @param source vector of POD type
 * @return resulting row matrix, matrix with single row pointing to the vector's underlying data
 */
template <typename T>
cv::Mat_<T> Vec_ToRowMat_(std::vector<T> &v)
{
    return cv::Mat_<T>(1, static_cast<int>(v.size()), v.data());
}

}

#endif // _ELM_CORE_MAT_VECTOR_UTILS_INL_H_

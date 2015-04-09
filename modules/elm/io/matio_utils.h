/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_IO_MATIO_UTILS_H_
#define _ELM_IO_MATIO_UTILS_H_

#ifdef __WITH_MATIO

#include <vector>

#include "matio.h"

#include "elm/core/cv/typedefs_fwd.h"   // cv::Mat fwd declaration

namespace elm {

/**
 * @brief map MATIO's MAT class enum to OpenCV's Mat type
 * @param MATIO's MAT class type
 * @return corresponding OpenCV's Mat type
 */
unsigned int MATIOClassTOCV_TYPE(matio_classes type);

/**
 * @brief Get copy of multi-dim slice extracted Mat object extracted from MAT file
 *
 * colum-major order is assumed
 *
 * @param[in] src matrix
 * @param[in] dim which dimension to slice at from source matrix
 * @param[in] idx index alongside dimension to slice from
 * @param[out] dst data of matrix slice copied into this object, Mat object's dims adjusted accordingly.
 */
void SliceCopy(const cv::Mat &src, int dim, int idx, cv::Mat& dst);

/**
 * @brief ind2sub get n-dmensional subscript from linear index
 *
 * Inspired by: https://bitbucket.org/kritisen/utilitiescpp/
 *
 * CAUTION: colum-major order is assumed
 *
 * @param idx linear index
 * @param dims  no. of dimensions
 * @param sizes size of each dimension
 * @param subs subscript across each dimension
 */
void ind2sub(int idx, int dims, const int *sizes, std::vector<int> &subs);

/**
 * @brief calculate linear index from subscript values
 * @param dims no. of dimensions
 * @param sizes size of each dimension
 * @param subs subscript of requested element across each dimension
 * @return linear index corresponding to multi-dimensional subscript
 */
int sub2ind(int dims, const int *sizes, const int *subs);

/**
 * @brief Convert 3-dimensional OpenCV Mat to multi-channel Mat
 * @param[in] src multi-dimensional Mat
 * @param[out] dst 2-dimensional multi-channel Mat
 */
void Mat3DTo3Ch(const cv::Mat &src, cv::Mat& dst);

} // namespace elm

#endif // __WITH_MATIO

#endif // _ELM_IO_MATIO_UTILS_H_

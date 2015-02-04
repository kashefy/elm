/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
/** Non-template utility definitions
 * For template inline definitions see *_inl*
  */
#ifndef _ELM_CORE_CV_MAT_UTILS_H_
#define _ELM_CORE_CV_MAT_UTILS_H_

#include "elm/core/typedefs_fwd.h"

namespace elm {

/**
 * @brief Converts matrix to 8U and scales elements
 * so that min=0 and max=255.
 * If all input elements are identical, yield matrix of zeros
 *
 * Useful for preparing matrix to use with highgui's cv::imshow()
 *
 * @param src
 * @return Mat_<uchar>
 */
cv::Mat_<unsigned char> ConvertTo8U(const cv::Mat &src);

/**
 * @brief Cumulative sum
 * @param src matrix
 * @param dst matrix
 */
void CumSum(const cv::Mat1f &src, cv::Mat1f &dst);

/**
 * @brief lower triangular part of matrix flattened
 *
 * Diagonal included.
 *
 * @param src
 * @param dst
 * @return total no. of elments extracted
 * @see tril()
 */
int tril_flat(const cv::Mat1f &src, cv::Mat1f &dst);

/**
 * @brief n-th order discrete forward diff along an axis.
 * @param src Mat
 * @param n no. of times values are differenced
 * @param axis along which to perform forward difference
 * @return resulting difference
 * @todo implement
 */
cv::Mat1f diff(const cv::Mat1f &src, int n=1, int axis=-1);

/**
  * @brief Convert 2d point of integers to single channel Mat of integers
  * @param 2d point of integers
  * @return single channel row-matrix of integers [x, y]
  */
cv::Mat1i Point2Mat(const cv::Point2i& p);

/**
 * @brief Convert matrix of integers into a  2-dimensional point
 * @param matrix (only first two elements will be considered)
 * @return point of matrix' first 2 elements
 * @throws elm::ExceptionBadDims if matrix contains less than 2 elements
 */
cv::Point2i Mat2Point2i(const cv::Mat1i &m);

/**
 * @brief Convert matrix of T (int or float) into a 3-dimensional point
 * @param matrix (only first 3 elements will be considered)
 * @return point of matrix' first 3 elements
 * @throws elm::ExceptionBadDims if matrix contains less than 3 elements
 */
template <typename T>
cv::Point3_<T> Mat2Point3_(const cv::Mat_<T> &m);

} // elm namespace

#endif // _ELM_CORE_CV_MAT_UTILS_H_

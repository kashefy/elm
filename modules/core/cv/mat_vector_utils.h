/** @file define routines involving STL vectors of Mat objects
  */
#ifndef SEM_CORE_CV_MAT_VECTOR_UTILS_H_
#define SEM_CORE_CV_MAT_VECTOR_UTILS_H_

#include "core/typedefs_sfwd.h"

namespace sem {

/**
 * @brief Get all element values at a position across different matrices
 * @param vector of matrices
 * @param row
 * @param col
 * @return row matrix with extracted elements
 * @throws ExceptionBadDims for positions that cannot be accessed.
 * @todo validate equally sized matrices inside vector or define protocol
 * current behavior, rely on dims of first vector entry
 */
cv::Mat1f ElementsAt(const VecMat1f &v, int row, int col);

/**
 * @brief Reshape vector of mat to single mat with row per element and col per vector element/block/kernel.
 * Only applicable to vector of equally sized matrices.
 * @param input vector of matrices
 * @return single matrix, row per matrix element and cols=vector size, all dims are non-zero
 * @todo enforce validation of same-dim matrix elements or define clear protocol, current behavior: rely on dims of first vector entry
 */
cv::Mat1f Reshape(const VecMat1f &v);

} // sem namespace

#endif // SEM_CORE_CV_MAT_VECTOR_UTILS_H_

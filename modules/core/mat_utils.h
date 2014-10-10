#ifndef SEM_CORE_MAT_UTILS_H_
#define SEM_CORE_MAT_UTILS_H_

#include <string>
#include "core/typedefs.h"

namespace sem {

/**
 * @brief Cumulative sum
 * @param src matrix
 * @param dst matrix
 */
void CumSum(const MatF &src, MatF &dst);

/**
 * @brief Get string representation of matrix type
 * @param matrix
 * @return type as string
 */
std::string MatTypeToString(const cv::Mat& m);

} // sem namespace

#endif // SEM_CORE_MAT_UTILS_H_

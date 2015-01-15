#ifndef SEM_CORE_CV_MAT_TYPE_UTILS_H_
#define SEM_CORE_CV_MAT_TYPE_UTILS_H_

#include "core/stl/typedefs.h"
#include "core/cv/typedefs_fwd.h"

namespace sem {

/**
 * @brief Get string representation of matrix type
 * @param matrix
 * @return type as string
 */
std::string MatTypeToString(const cv::Mat& m);

} // namespace sem

#endif // SEM_CORE_CV_MAT_TYPE_UTILS_H_

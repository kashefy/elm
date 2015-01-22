#ifndef ELM_CORE_CV_MAT_TYPE_UTILS_H_
#define ELM_CORE_CV_MAT_TYPE_UTILS_H_

#include "sem/core/stl/typedefs.h"
#include "sem/core/cv/typedefs_fwd.h"

namespace elm {

/**
 * @brief Get string representation of matrix type
 * @param matrix
 * @return type as string
 */
std::string MatTypeToString(const cv::Mat& m);

} // namespace elm

#endif // ELM_CORE_CV_MAT_TYPE_UTILS_H_

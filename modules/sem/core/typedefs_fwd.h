/** @file near-fwd declarations for convinience typedefs
 * Not fully forward due to STL types.
 */
#ifndef SEM_CORE_TYPEDEFS_FWD_H_
#define SEM_CORE_TYPEDEFS_FWD_H_

#include "sem/core/cv/typedefs_fwd.h"

namespace sem {

typedef cv::Mat_<float> Mat_f;   ///< convinience forward typedef for Mat of floats without constraints on no. of channels

} // namespace sem

#endif // SEM_CORE_TYPEDEFS_FWD_H_

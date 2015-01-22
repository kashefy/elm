/** @file near-fwd declarations for convinience typedefs
 * Not fully forward due to STL types.
 */
#ifndef ELM_CORE_TYPEDEFS_FWD_H_
#define ELM_CORE_TYPEDEFS_FWD_H_

#include "sem/core/cv/typedefs_fwd.h"

typedef unsigned char uchar;

namespace elm {

typedef cv::Mat_<float> Mat_f;   ///< convinience forward typedef for Mat of floats without constraints on no. of channels

} // namespace elm

#endif // ELM_CORE_TYPEDEFS_FWD_H_

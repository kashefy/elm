/** @file define elmi-forwarded typedef. Fully defined STL types + fwd. declared other types
 */
#ifndef ELM_CORE_TYPEDEFS_SFWD_H_
#define ELM_CORE_TYPEDEFS_SFWD_H_

#include "sem/core/typedefs_fwd.h"
#include "sem/core/stl/typedefs.h"

namespace elm {

typedef std::vector< cv::Mat1f > VecMat1f;  ///< Convinience typedef for vector of single channel float matrices
typedef std::vector< cv::Mat > VecMat;      ///< Convinience typedef for vector of matrices

} // namespace elm

#endif // ELM_CORE_TYPEDEFS_SFWD_H_

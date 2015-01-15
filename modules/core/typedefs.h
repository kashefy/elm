#ifndef SEM_CORE_TYPEDEFS_H_
#define SEM_CORE_TYPEDEFS_H_

#include "core/typedefs_fwd.h"
#include "core/stl/typedefs.h"

namespace sem {

typedef std::vector< cv::Mat1f > VecMat1f;  ///< Convinience typedef for vector of single channel float matrices
typedef std::vector< cv::Mat > VecMat;      ///< Convinience typedef for vector of matrices

} // namespace sem

#endif // SEM_CORE_TYPEDEFS_H_

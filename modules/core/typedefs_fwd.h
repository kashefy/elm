/** @file near-fwd declarations for convinience typedefs
  */
#ifndef SEM_CORE_TYPEDEFS_FWD_H_
#define SEM_CORE_TYPEDEFS_FWD_H_

#include "core/stl/typedefs.h"
#include "core/cv/typedefs_fwd.h"

namespace sem {

typedef cv::Mat_<float> Mat_f;   ///< convinience typedef for Mat of floats without constrains on no. of channels

typedef std::vector< cv::Mat1f > VecMat1f;  ///< Convinience typedef for vector of single channel float matrices
typedef std::vector< cv::Mat1f > VecMat;    ///< Convinience typedef for vector of matrices

} // namespace sem

#endif // SEM_CORE_TYPEDEFS_FWD_H_

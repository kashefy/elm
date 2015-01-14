/** forward declarations of boost data types
 */
#ifndef SEM_CORE_OPENCV_TYPEDEFS_FWD_H_
#define SEM_CORE_OPENCV_TYPEDEFS_FWD_H_

#include "core/stl/typedefs.h"
#include <opencv2/core.hpp>

namespace cv {

// only types that are defined by OpenCV
class Mat;

template <typename T> class Mat_;

typedef Mat_<float> Mat1f;   ///< convinience typedef for Mat of floats without constrains on no. of channels

template <typename T> class Point_;

typedef Point_<int> Point2i;
typedef Point_<float> Point2f;
typedef Point2i Point;

} // fake namespace cv for fwd declaration

#endif // SEM_CORE_OPENCV_TYPEDEFS_FWD_H_

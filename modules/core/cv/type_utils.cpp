#include "core/cv/type_utils.h"

#include <opencv2/core.hpp>

using namespace std;
using namespace cv;

string sem::MatTypeToString(const Mat& m)
{
    string type_name;
    uchar depth = m.type() & CV_MAT_DEPTH_MASK;

    switch ( depth ) {
      case CV_8U:   type_name = "CV_8U"  ; break;
      case CV_8S:   type_name = "CV_8S"  ; break;
      case CV_16U:  type_name = "CV_16U" ; break;
      case CV_16S:  type_name = "CV_16S" ; break;
      case CV_32S:  type_name = "CV_32S" ; break;
      case CV_32F:  type_name = "CV_32F" ; break;
      case CV_64F:  type_name = "CV_64F" ; break;
      default: break;
    }
    return type_name;
}

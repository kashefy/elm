#ifndef SEM_CORE_TYPEDEFS_H_
#define SEM_CORE_TYPEDEFS_H_

#include <vector>
#include <opencv2/core.hpp>

typedef cv::Mat_<float> MatF;
typedef std::vector< MatF > VecMatF;
typedef cv::Mat_<int> MatI;

#endif // SEM_CORE_TYPEDEFS_H_

#ifndef SEM_CORE_TYPEDEFS_H_
#define SEM_CORE_TYPEDEFS_H_

#include <vector>
#include <string>
#include <opencv2/core.hpp>

typedef std::vector< std::string > VecS;    ///< Convinience typedef for vector of strings
typedef std::vector< float > VecF;          ///< Convinience typedef for vector of floats
typedef std::vector< cv::Mat1f > VecMat1f;  ///< Convinience typedef for vector of single channel float matrices
typedef std::vector< cv::Mat1f > VecMat;    ///< Convinience typedef for vector of matrices
typedef VecMat1f::const_iterator VecMat1fCIter; ///< Convinience typedef for VecMat1f const iterator
typedef VecMat1f::const_iterator VecMat1fIter;  ///< Convinience typedef for VecMat1f iterator

typedef cv::Mat_<float> Mat_f;   ///< convinience typedef for Mat of floats without constrains on no. of channels

#endif // SEM_CORE_TYPEDEFS_H_

/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
/** forward typedef of OpenCV data types
 */
#ifndef _ELM_CORE_OPENCV_TYPEDEFS_FWD_H_
#define _ELM_CORE_OPENCV_TYPEDEFS_FWD_H_

namespace cv {

// only typedefs already provided by OpenCV
class Mat;

template <typename T> class Mat_;
template <typename T> class SparseMat_;

typedef Mat_<float> Mat1f;  ///< convinience typedef for Mat of floats without constraints on no. of channels
typedef Mat_<int> Mat1i;    ///< convinience typedef for Mat of integers

template <typename T, int cn> class Vec; ///< convinience typedef for Vec used in multi-channel pixel representation

template <typename T> class Point_; ///< convinience typedef for point template
/**
 * @typedef 2d points
 */
typedef Point_<int> Point2i;    ///< convinience typedef for 2d point of integers
typedef Point_<float> Point2f;  ///< convinience typedef for 2d point of floats
typedef Point2i Point;          ///< extra-convinience typedef for 2d point of integers

template<typename _Tp> class Point3_;
/**
 * @typedef 3d points
 */
typedef Point3_<int> Point3i;       ///< convinience typedef for 3d point of integers
typedef Point3_<float> Point3f;     ///< convinience typedef for 3d point of floats
typedef Point3_<double> Point3d;    ///< convinience typedef for 3d point of doubles - not preferred

} // fake namespace cv for fwd declaration

#endif // _ELM_CORE_OPENCV_TYPEDEFS_FWD_H_

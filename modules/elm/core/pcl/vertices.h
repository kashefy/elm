/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, Elm Library Project
// 3-clause BSD License
//
//M*/
/** A collection of routines to facilitate working with PCL data types.
 * Only defined if PCL support exists.
 */
#ifndef _ELM_CORE_PCL_UTILS_H_
#define _ELM_CORE_PCL_UTILS_H_

#ifndef __WITH_PCL
    #warning "Skipping pcl utilities since PCL support is disabled."
#else // __WITH_PCL

#include <boost/shared_ptr.hpp>
#include <pcl/Vertices.h>

#include "elm/core/cv/typedefs_fwd.h"
#include "elm/core/pcl/typedefs_fwd.h"

namespace elm {

/**
 * @brief Convert Mat of floats to vector of pcl Vertices
 *
 * Unfortunately, a shallow copy did not work, this will be a deep copy.
 *
 * @param Mat with vertices info
 * @return vector of verrtices
 *
 * @todo We know how to convert a Mat_<T1> -> STL vector<T1> but for this we need to convert a Mat_<T1> -> STL vector<T2!>
 */
VecVertices Mat2VecVertices(const cv::Mat &m);

/**
 * @brief Convert vector of Vertices to a Mat of floats
 *
 * Unfortunately, a shallow copy did not work, this will be a deep copy
 *
 * @param vector of vertices
 * @param whether to format the resulting Mat as a row matrix or not
 * @return Mat of floats with vertex data
 */
cv::Mat1f VecVertices2Mat(const VecVertices& vv, bool do_row_mat=true);

}

#endif // __WITH_PCL

#endif // _ELM_CORE_PCL_UTILS_H_

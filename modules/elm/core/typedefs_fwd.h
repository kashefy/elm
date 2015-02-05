/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
/** @file near-fwd declarations for convinience typedefs
 * Not fully forward due to STL types.
 */
#ifndef _ELM_CORE_TYPEDEFS_FWD_H_
#define _ELM_CORE_TYPEDEFS_FWD_H_

#include "elm/core/boost/typedefs_fwd.h"
#include "elm/core/cv/typedefs_fwd.h"

typedef unsigned char uchar;

namespace elm {

typedef cv::Mat_<float> Mat_f;   ///< convinience forward typedef for Mat of floats without constraints on no. of channels

typedef boost::property_tree::ptree PTree;

} // namespace elm

#endif // _ELM_CORE_TYPEDEFS_FWD_H_

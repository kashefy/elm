/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
/** @file define semi-forwarded typedef. Fully defined STL types + fwd. declared other types
 */
#ifndef _ELM_CORE_TYPEDEFS_SFWD_H_
#define _ELM_CORE_TYPEDEFS_SFWD_H_

#include "elm/core/typedefs_fwd.h"
#include "elm/core/stl/typedefs.h"

namespace elm {

typedef std::vector< cv::Mat1f > VecMat1f;  ///< Convinience typedef for vector of single channel float matrices
typedef std::vector< cv::Mat > VecMat;      ///< Convinience typedef for vector of matrices

typedef std::map<std::string, std::string> MapSS;   ///< Convinience typedef for map of string keys and string values

typedef boost::optional<std::string> OptS;  ///< Convinience typedef for optional string

} // namespace elm

#endif // _ELM_CORE_TYPEDEFS_SFWD_H_

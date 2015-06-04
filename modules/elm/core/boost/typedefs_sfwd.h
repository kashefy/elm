/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
/** @file define semi-forwarded typedef. Fully defined STL types + fwd. declared other types
 */
#ifndef _ELM_CORE_BOOST_TYPEDEFS_SFWD_H_
#define _ELM_CORE_BOOST_TYPEDEFS_SFWD_H_

#include <string>

#include "elm/core/typedefs_fwd.h"

namespace elm {

typedef boost::optional<std::string> OptS;  ///< Convinience typedef for optional string

} // namespace elm

#endif // _ELM_CORE_BOOST_TYPEDEFS_SFWD_H_

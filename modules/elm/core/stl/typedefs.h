/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
/** forward declarations of STL data types
 */
#ifndef _ELM_CORE_STL_TYPEDEFS_H_
#define _ELM_CORE_STL_TYPEDEFS_H_

#include <map>
#include <string>
#include <vector>

namespace elm {

typedef unsigned char uchar;

typedef std::vector< std::string > VecS;    ///< Convinience typedef for vector of strings
typedef std::vector< float > VecF;          ///< Convinience typedef for vector of floats
typedef std::map<std::string, std::string> MapSS;   ///< Convinience typedef for map of string keys and string values

}

#endif // _ELM_CORE_STL_TYPEDEFS_FWD_H_

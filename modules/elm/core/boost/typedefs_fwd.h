/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
/** forward declarations of boost data types
 */
#ifndef _ELM_CORE_BOOST_TYPEDEFS_FWD_H_
#define _ELM_CORE_BOOST_TYPEDEFS_FWD_H_

#include <boost/property_tree/ptree_fwd.hpp>

namespace boost {

template <typename T> class shared_ptr;

} // fake namespace boost for fwd declarations

namespace elm {

typedef boost::property_tree::ptree PTree;

} // namespace elm

#endif // _ELM_CORE_PCL_TYPEDEFS_FWD_H_

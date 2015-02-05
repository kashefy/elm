/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
/** @file utilities around boost property tree with inline definitions
 * or that need full type specification
 */
#ifndef _ELM_CORE_BOOST_PTREE_UTILS_INL_H_
#define _ELM_CORE_BOOST_PTREE_UTILS_INL_H_

#include <vector>

#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>

#include "elm/core/typedefs_fwd.h"

namespace elm
{

/**
 * @brief Print a ptree to standard out in xml format
 * @param[in] property tree to print
 * @param[out] output stream, cout by default
 */
void PrintXML(const PTree &pt,
              std::basic_ostream<
              typename PTree::key_type::value_type
              > &stream=std::cout);

/**
 * @brief push back elements from property_tree into a vector
 * @param[in] source tree
 * @param[in] node key
 * @param[out] vector to push elements into
 * @throws boost::property_tree::ptree_bad_path for keys that do not exist
 */
template <class T>
void push_back_child(const PTree &p, const std::string &key, std::vector<T> &v)
{
    BOOST_FOREACH(const PTree::value_type &node, p.get_child(key)) {

        v.push_back(node.second.get_value<T>());
    }
}

} // namespace elm

#endif // _ELM_CORE_BOOST_PTREE_UTILS_INL_H_

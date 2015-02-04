/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
/** @file utilities around boost property tree
 */
#ifndef _ELM_CORE_PTREE_UTILS_H_
#define _ELM_CORE_PTREE_UTILS_H_

#include <iosfwd>
#include <iostream>
#include <vector>

#include "elm/core/typedefs_fwd.h"

namespace elm
{

/**
 * @brief Check PTree for unsued nodes
 * Suitable for checking a layer is not getting extra paramters
 * @param[in] PTree to check for unused nodes
 * @param[in] keys expected to be found
 * @param[in] unused keys, keys found in addition to expected set of keys
 * @return true if unusued keys found
 */
bool UnusedNodes(const PTree &p, const std::vector<std::string> &keys_used, std::vector<std::string> &keys_unused);

}

#endif // _ELM_CORE_PTREE_UTILS_H_

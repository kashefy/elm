/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
/** @file utilities around boost property tree
 */
#ifndef _ELM_CORE_BOOST_PTREE_UTILS_H_
#define _ELM_CORE_BOOST_PTREE_UTILS_H_

#include "elm/core/typedefs_sfwd.h"

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
bool UnusedNodes(const PTree &p, const VecS &keys_used, VecS &keys_unused);

/**
 * @brief Convert property tree to map
 * @param[in] src source property tree
 * @param[out] dst map
 */
void PTreeToMapSS(const PTree &src, MapSS &dst);

/**
 * @brief Convert map to property tree
 * @param[in] src map
 * @param[out] dst property tree
 */
void MapSSToPTree(const MapSS &src, PTree &dst);

}

#endif // _ELM_CORE_BOOST_PTREE_UTILS_H_

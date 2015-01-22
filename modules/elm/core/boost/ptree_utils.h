/** @file utilities around boost property tree
 */
#ifndef ELM_CORE_PTREE_UTILS_H_
#define ELM_CORE_PTREE_UTILS_H_

#include <iostream>
#include <vector>

#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>

namespace elm
{
typedef boost::property_tree::ptree PTree;

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

#endif // ELM_CORE_PTREE_UTILS_H_

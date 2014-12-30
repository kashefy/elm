/** @file utilities around boost property tree
 */
#ifndef SEM_CORE_PTREE_UTILS_H_
#define SEM_CORE_PTREE_UTILS_H_

#include <iostream>

#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>

namespace sem
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

template <class T>
void push_back_child(const PTree &p, const std::string &key, std::vector<T> &v)
{
    BOOST_FOREACH(const PTree::value_type &node, p.get_child(key)) {

        v.push_back(node.second.get_value<T>());
    }
}

}

#endif // SEM_CORE_PTREE_UTILS_H_

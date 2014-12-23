/** @file utilities around boost property tree
 */
#ifndef SEM_CORE_PTREE_UTILS_H_
#define SEM_CORE_PTREE_UTILS_H_

#include <iostream>
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

}

#endif // SEM_CORE_PTREE_UTILS_H_

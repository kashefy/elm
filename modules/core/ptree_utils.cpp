#include "core/ptree_utils.h"

#include <iostream>

#include <boost/property_tree/xml_parser.hpp>

using namespace std;
using namespace sem;

using namespace boost::property_tree;

void sem::PrintXML(const PTree &pt,
                std::basic_ostream<
                typename PTree::key_type::value_type
                > &stream)
{
    xml_parser::write_xml(stream, pt);
}


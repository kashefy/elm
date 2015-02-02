#include "elm/core/boost/ptree_utils_inl.h"

#include <boost/property_tree/xml_parser.hpp>

using namespace boost::property_tree;

void elm::PrintXML(const PTree &pt,
                   std::basic_ostream<
                   typename PTree::key_type::value_type
                   > &stream)
{
    xml_parser::write_xml(stream, pt);
}

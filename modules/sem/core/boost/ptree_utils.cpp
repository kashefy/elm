#include "sem/core/boost/ptree_utils.h"

#include <algorithm>

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

bool sem::UnusedNodes(const PTree &p, const vector<string> &keys_used, vector<string> &keys_unused)
{
    for(PTree::const_iterator itr=p.begin(); itr != p.end(); ++itr) {

        string tmp = (*itr).first;
        if(find(keys_used.begin(), keys_used.end(), tmp) == keys_used.end()) { // not found

            keys_unused.push_back(tmp);
        }
    }
    return keys_unused.size() > 0;
}

/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/core/boost/ptree_utils.h"

#include <algorithm>

#include <boost/property_tree/ptree.hpp>

using namespace std;
using namespace elm;

using namespace boost::property_tree;

bool elm::UnusedNodes(const PTree &p, const vector<string> &keys_used, vector<string> &keys_unused)
{
    for(PTree::const_iterator itr=p.begin(); itr != p.end(); ++itr) {

        string tmp = (*itr).first;
        if(find(keys_used.begin(), keys_used.end(), tmp) == keys_used.end()) { // not found

            keys_unused.push_back(tmp);
        }
    }
    return keys_unused.size() > 0;
}

void elm::PTreeToMapSS(const PTree &src, MapSS &dst) {

    for(PTree::const_iterator itr=src.begin(); itr != src.end(); ++itr) {

        string key = (*itr).first;
        dst[key] = src.get<std::string>(key);
    }
}

void elm::MapSSToPTree(const MapSS &src, PTree &dst) {

    for(MapSS::const_iterator itr=src.begin(); itr != src.end(); ++itr) {

        dst.put(itr->first, itr->second);
    }
}

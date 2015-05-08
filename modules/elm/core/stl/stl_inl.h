/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
/**@file collection of STL utility/convinience functions
 * defined inline
 */
#ifndef _ELM_CORE_STL_INL_H_
#define _ELM_CORE_STL_INL_H_

#include <map>
#include <stdlib.h> // rand
#include <string>
#include <sstream>
#include <vector>

namespace elm {

    /**
     * @brief Find key in map and provide corresponding object if exists.
     * @param[in] map to search
     * @param[in] key to search for
     * @param[out] object if exists
     * @return true if found
     */
    template <class T>
    bool Find(const std::map<std::string, T > &map, const std::string &key, T &obj)
    {
        typename std::map<std::string, T >::const_iterator itr = map.find(key);
        bool found = itr != map.end();
        if(found) {

            obj = itr->second;
        }
        return found;
    }

    /**
     * @brief List all keys in a map
     * @param map
     * @return vector of all map keys
     */
    template <class TKey, class TVal>
    std::vector<TKey > Keys(const std::map<TKey, TVal > &m)
    {
        std::vector<TKey > v(m.size());

        typename std::map<TKey, TVal >::const_iterator itr;
        int i=0;
        for(itr=m.begin(); itr != m.end(); ++itr) {

            v[i++] = itr->first;
        }
        return v;
    }

    /**
     * @brief List all values in a map
     * @param map
     * @return vector of all map values
     */
    template <class TKey, class TVal>
    std::vector<TVal > Values(const std::map<TKey, TVal > &m)
    {
        std::vector<TVal > v(m.size());

        typename std::map<TKey, TVal >::const_iterator itr;
        int i=0;
        for(itr=m.begin(); itr != m.end(); ++itr) {

            v[i++] = itr->second;
        }
        return v;
    }

    /**
     * @brief Push back uniformly random values into a vector
     * @param vector pushing back into
     * @param no. of items to push back
     */
    template <typename T>
    void push_back_randu(std::vector<T> &v, int N)
    {
        for(int i=0; i<N; i++) {

            v.push_back(static_cast<T>(rand()));
        }
    }

    template <typename T>
    std::string to_string(const std::vector<T> &v, const std::string &delim=", ")
    {
        std::stringstream s;
        for(size_t i=0; i<v.size(); i++) {

            s << v[i];
            if(i < v.size()-1) {

                s << delim;
            }
        }

        return s.str();
    }
}

#endif // _ELM_CORE_STL_INL_H_

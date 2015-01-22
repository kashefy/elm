/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, Elm Library Project
// 3-clause BSD License
//
//M*/
/**@file collection of STL utility/convinience functions
 */
#ifndef ELM_CORE_STL_H_
#define ELM_CORE_STL_H_

#include <map>
#include <string>
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
}

#endif // ELM_CORE_STL_H_

/**@file collection of STL utility/convinience functions
 */
#ifndef SEM_CORE_STL_H_
#define SEM_CORE_STL_H_

#include <map>
#include <string>
#include <vector>

namespace sem {

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
        std::vector<TKey > v;

        typename std::map<TKey, TVal >::const_iterator itr;
        for(itr=m.begin(); itr != m.end(); ++itr) {

            v.push_back(itr->first);
        }
        return v;
    }
}

#endif // SEM_CORE_STL_H_

/**@file collection of STL utility/convinience functions
 */
#ifndef SEM_CORE_STL_H_
#define SEM_CORE_STL_H_

#include <map>

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
}

#endif // SEM_CORE_STL_H_

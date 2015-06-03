#ifndef _ELM_PYTHON_STL_INL_H_
#define _ELM_PYTHON_STL_INL_H_

#include <vector>

#include <boost/python/list.hpp>

namespace elm {

/** @brief Convert an C++ STL vector to a python list
 * credits: https://gist.github.com/octavifs/5362272
 * @param v vector
 * @return list
 */
template <class T>
boost::python::list toPythonList(std::vector<T> v) {

    typename std::vector<T>::iterator itr;
    boost::python::list list;
    for (itr = v.begin(); itr != v.end(); ++itr) {

        list.append(*itr);
    }
    return list;
}

} // namespace elm

#endif // _ELM_PYTHON_STL_INL_H_


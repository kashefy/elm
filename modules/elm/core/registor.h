/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_CORE_REGISTOR_H_
#define _ELM_CORE_REGISTOR_H_

#include <map>
#include <memory>
#include <string>

#include "elm/core/exception.h"
#include "elm/core/stl/stl.h"

/** @brief class for registering derived instantances
 * Partially inspired by:
 * http://stackoverflow.com/questions/582331/is-there-a-way-to-instantiate-objects-from-a-string-holding-their-class-name
 */
template <class TBase>
class Registor_
{
public:
    typedef std::string RegisteredTypeKey;      ///< convinience for name of instance type
    typedef std::shared_ptr<TBase> RegisteredTypeSharedPtr;    ///< convinience for shared pointer to instance
    typedef std::map< RegisteredTypeKey, RegisteredTypeSharedPtr(*)() > Registry;    ///< convinience for registry map

    /**
     * @brief Factory method for instantiating a shared pointer to an instance
     * @param requested type
     * @return pointer to new instance
     * @throws ExceptionTypeError on unrecognized type
     */
    static RegisteredTypeSharedPtr CreatePtrShared(const Registry &r, const RegisteredTypeKey &type) {

        RegisteredTypeSharedPtr ptr;

        typename Registry::const_iterator itr = r.find(type);
        bool found = itr != r.end();
        if(found) {

            ptr = itr->second();
        }
        else {

            std::stringstream s;
            s << "\"" << type << "\" is not a registered type.";
            ELM_THROW_TYPE_ERROR(s.str());
        }
        return ptr;
    }

    template <class TDerived>
    static RegisteredTypeSharedPtr DerivedInstance() {

        return RegisteredTypeSharedPtr(new TDerived());
    }

    /**
     * @brief Find type in registry
     * Effectively a wrapper around STL Map's find routine
     * @param registry
     * @param type to search for
     * @return true if found successfully
     */
    static bool Find(const Registry &r, const RegisteredTypeKey &type) {

        return r.find(type) != r.end();
    }
};

#endif // _ELM_CORE_REGISTOR_H_

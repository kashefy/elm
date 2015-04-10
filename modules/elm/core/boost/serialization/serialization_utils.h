/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_CORE_BOOST_SERIALIZATION_SERIALIZATION_UTILS_H_
#define _ELM_CORE_BOOST_SERIALIZATION_SERIALIZATION_UTILS_H_

namespace elm {

namespace detail {

template<class Tarchive, class TObj>
void Save(Tarchive &ar, const TObj &obj)
{
    ar & obj;
}

template<class Tarchive, class TObj>
void Load(Tarchive &ar, TObj &obj)
{
    ar & obj;
}

} // namespace detail

} // namespace elm

#endif // _ELM_CORE_BOOST_SERIALIZATION_SERIALIZATION_UTILS_H_

/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_CORE_BOOST_SERIALIZATION_SER_CVTERMCRITERIA_H_
#define _ELM_CORE_BOOST_SERIALIZATION_SER_CVTERMCRITERIA_H_
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/nvp.hpp>

#include <opencv2/core/types_c.h>

namespace boost {
namespace serialization {

    template<class Tarchive>
    void serialize(Tarchive &ar, CvTermCriteria &obj, const unsigned int version)
    {
        using namespace boost::serialization;
        ar & make_nvp("epsilon",    obj.epsilon);
        ar & make_nvp("max_iter",   obj.max_iter);
        ar & make_nvp("type",       obj.type);
    }

} // namespace boost
} // namespace serialization

#endif // _ELM_CORE_BOOST_SERIALIZATION_SER_CVTERMCRITERIA_H_

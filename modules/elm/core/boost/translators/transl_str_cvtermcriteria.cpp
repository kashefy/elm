/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/core/boost/translators/transl_str_cvtermcriteria.h"

#include <sstream>

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>

#include "elm/core/boost/serialization/ser_cvtermcriteria.h"

using namespace boost;
namespace ba=boost::archive;
using namespace elm;

optional<translator_CvTermCriteria::external_type>
translator_CvTermCriteria::get_value(const internal_type& str)
{
    if (!str.empty())
    {
        std::istringstream stream(str);

        ba::text_iarchive ia(stream,
                             ba::no_header     |
                             ba::no_tracking   |
                             ba::no_codecvt    |
                             ba::no_xml_tag_checking);

        external_type x;
        ia >> x;

        return optional<external_type>(x);
    }
    else {

        return optional<external_type>(boost::none);
    }
}

// Create a string from a low_high_value
boost::optional<translator_CvTermCriteria::internal_type>
translator_CvTermCriteria::put_value(const external_type& x)
{
    std::ostringstream stream;
    ba::text_oarchive oa(stream,
                         ba::no_header |
                         ba::no_tracking   |
                         ba::no_codecvt    |
                         ba::no_xml_tag_checking);
    oa << x;
    return optional<internal_type>(stream.str());
}

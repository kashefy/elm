/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_CORE_BOOST_TRANSLATORS_TRANSL_STR_MAPSS_H_
#define _ELM_CORE_BOOST_TRANSLATORS_TRANSL_STR_MAPSS_H_

#include <string>
#include <map>

#include <boost/property_tree/ptree.hpp>

extern template class std::map<std::string, std::string>;

namespace elm {

/**
 * @brief STL string <-> Map<string,string> translator
 * for adding MapSS objects to boost property trees
 */
struct translator_MapSS
{
  typedef std::string    internal_type;
  typedef std::map<std::string, std::string> external_type;

  boost::optional<external_type> get_value(const internal_type& str);

  // Create a string from a low_high_value
  boost::optional<internal_type> put_value(const external_type& x);
};

} // namespace elm

namespace boost {

namespace property_tree {

    template<typename Ch, typename Traits, typename Alloc>
    struct translator_between<std::basic_string<Ch, Traits, Alloc >, std::map<std::string, std::string> >
    {
        typedef elm::translator_MapSS type;
    };

} // namespace property_tree
} // namespace boost

#endif // _ELM_CORE_BOOST_TRANSLATORS_TRANSL_STR_MAPSS_H_

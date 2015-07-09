/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_CORE_BOOST_TRANSLATORS_TRANSL_STR_VECI_H_
#define _ELM_CORE_BOOST_TRANSLATORS_TRANSL_STR_VECI_H_

#include <string>
#include <vector>

#include <boost/property_tree/ptree.hpp>

namespace elm {

/**
 * @brief STL string <-> OpenCV CvTermCriteria translator
 * for adding CvTermCriteria to boost property trees
 */
struct translator_VecI
{
  typedef std::string    internal_type;
  typedef std::vector<int> external_type;

  boost::optional<external_type> get_value(const internal_type& str);

  // Create a string from a low_high_value
  boost::optional<internal_type> put_value(const external_type& x);
};

} // namespace elm


namespace boost {

namespace property_tree {

    template<typename Ch, typename Traits, typename Alloc>
    struct translator_between<std::basic_string<Ch, Traits, Alloc >, std::vector<int> >
    {
        typedef elm::translator_VecI type;
    };

} // namespace property_tree
} // namespace boost

#endif // _ELM_CORE_BOOST_TRANSLATORS_TRANSL_STR_VECI_H_

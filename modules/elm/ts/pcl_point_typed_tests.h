/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
/** @file utilitites for setting up typed tests using PCL Point
  */
#ifndef _ELM_TS_PCL_POINT_TYPED_TESTS_H_
#define _ELM_TS_PCL_POINT_TYPED_TESTS_H_

#ifdef __WITH_PCL // the following definition require PCL support

#include "gtest/gtest-typed-test.h"

#include <pcl/common/io.h>



typedef ::testing::Types<pcl::PointXYZ, pcl::Normal, pcl::PointNormal> PCLPointTypes;

namespace elm {

namespace ts {

/** @brief helper struct with expected attributes per point type
 */
template <typename TPoint>
struct ExpectedPointAttr_
{
    static const size_t field_count;
    static const size_t nb_floats;
    static const std::string name;
};

#define ELM_DECLARE_SPECIALIZATION_EXPECTED_POINT_FIELD_COUNT(TPoint)   template<> const size_t elm::ts::ExpectedPointAttr_<TPoint>::field_count
#define ELM_DECLARE_SPECIALIZATION_EXPECTED_POINT_NB_FLOAT(TPoint)      template<> const size_t elm::ts::ExpectedPointAttr_<TPoint>::nb_floats
#define ELM_DECLARE_SPECIALIZATION_EXPECTED_POINT_NAME(TPoint)          template<> const std::string elm::ts::ExpectedPointAttr_<TPoint>::name

/** Declare specialization of expected attributes for a Point type used in tests below.
 *  @param TPoint point type
 *  @param c field count    (no. of floats without padding)
 *  @param n no. of floats occupied (no. of floats with padding)
  */
#define ELM_DECLARE_SPECIALIZATION_EXPECTED_POINT_ATTR(TPoint) ELM_DECLARE_SPECIALIZATION_EXPECTED_POINT_NAME(TPoint); ELM_DECLARE_SPECIALIZATION_EXPECTED_POINT_FIELD_COUNT(TPoint); ELM_DECLARE_SPECIALIZATION_EXPECTED_POINT_NB_FLOAT(TPoint)

// Declare specializations
ELM_DECLARE_SPECIALIZATION_EXPECTED_POINT_ATTR(pcl::PointXYZ);
ELM_DECLARE_SPECIALIZATION_EXPECTED_POINT_ATTR(pcl::Normal);
ELM_DECLARE_SPECIALIZATION_EXPECTED_POINT_ATTR(pcl::PointNormal);

#define ELM_SET_EXPECTED_POINT_FIELD_COUNT(TPoint, c)   template<> const size_t elm::ts::ExpectedPointAttr_<TPoint>::field_count(c)
#define ELM_SET_EXPECTED_POINT_NB_FLOAT(TPoint, n)  template<> const size_t elm::ts::ExpectedPointAttr_<TPoint>::nb_floats(n)
#define ELM_SET_EXPECTED_POINT_NAME(TPoint)         template<> const std::string elm::ts::ExpectedPointAttr_<TPoint>::name(#TPoint)

/** Set expected attributes for a Point type used in tests below.
 *  @param TPoint point type
 *  @param c field count    (no. of floats without padding)
 *  @param n no. of floats occupied (no. of floats with padding)
  */
#define ELM_SET_EXPECTED_POINT_ATTR(TPoint, c, n) ELM_SET_EXPECTED_POINT_NAME(TPoint); ELM_SET_EXPECTED_POINT_FIELD_COUNT(TPoint, c); ELM_SET_EXPECTED_POINT_NB_FLOAT(TPoint, n)

} // namespace ts

} // namespace elm

#endif // __WITH_PCL

#endif // _ELM_TS_PCL_POINT_TYPED_TESTS_H_

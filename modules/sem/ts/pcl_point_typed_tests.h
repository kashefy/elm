/** @file utilitites for setting up typed tests using PCL Point
  */
#ifndef SEM_TS_PCL_POINT_TYPED_TESTS_H_
#define SEM_TS_PCL_POINT_TYPED_TESTS_H_

#ifdef __WITH_PCL // the following definition require PCL support

#include "gtest/gtest-typed-test.h"

#include <pcl/common/io.h>



typedef ::testing::Types<pcl::PointXYZ, pcl::Normal, pcl::PointNormal> PCLPointTypes;

namespace sem {

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

#define SEM_DECLARE_SPECIALIZATION_EXPECTED_POINT_FIELD_COUNT(TPoint)   template<> const size_t sem::ts::ExpectedPointAttr_<TPoint>::field_count
#define SEM_DECLARE_SPECIALIZATION_EXPECTED_POINT_NB_FLOAT(TPoint)      template<> const size_t sem::ts::ExpectedPointAttr_<TPoint>::nb_floats
#define SEM_DECLARE_SPECIALIZATION_EXPECTED_POINT_NAME(TPoint)          template<> const std::string sem::ts::ExpectedPointAttr_<TPoint>::name

/** Declare specialization of expected attributes for a Point type used in tests below.
 *  @param TPoint point type
 *  @param c field count    (no. of floats without padding)
 *  @param n no. of floats occupied (no. of floats with padding)
  */
#define SEM_DECLARE_SPECIALIZATION_EXPECTED_POINT_ATTR(TPoint) SEM_DECLARE_SPECIALIZATION_EXPECTED_POINT_NAME(TPoint); SEM_DECLARE_SPECIALIZATION_EXPECTED_POINT_FIELD_COUNT(TPoint); SEM_DECLARE_SPECIALIZATION_EXPECTED_POINT_NB_FLOAT(TPoint)

// Declare specializations
SEM_DECLARE_SPECIALIZATION_EXPECTED_POINT_ATTR(pcl::PointXYZ);
SEM_DECLARE_SPECIALIZATION_EXPECTED_POINT_ATTR(pcl::Normal);
SEM_DECLARE_SPECIALIZATION_EXPECTED_POINT_ATTR(pcl::PointNormal);

#define SEM_SET_EXPECTED_POINT_FIELD_COUNT(TPoint, c)   template<> const size_t sem::ts::ExpectedPointAttr_<TPoint>::field_count(c)
#define SEM_SET_EXPECTED_POINT_NB_FLOAT(TPoint, n)  template<> const size_t sem::ts::ExpectedPointAttr_<TPoint>::nb_floats(n)
#define SEM_SET_EXPECTED_POINT_NAME(TPoint)         template<> const std::string sem::ts::ExpectedPointAttr_<TPoint>::name(#TPoint)

/** Set expected attributes for a Point type used in tests below.
 *  @param TPoint point type
 *  @param c field count    (no. of floats without padding)
 *  @param n no. of floats occupied (no. of floats with padding)
  */
#define SEM_SET_EXPECTED_POINT_ATTR(TPoint, c, n) SEM_SET_EXPECTED_POINT_NAME(TPoint); SEM_SET_EXPECTED_POINT_FIELD_COUNT(TPoint, c); SEM_SET_EXPECTED_POINT_NB_FLOAT(TPoint, n)

} // namespace ts

} // namespace sem

#endif // __WITH_PCL

#endif // SEM_TS_PCL_POINT_TYPED_TESTS_H_

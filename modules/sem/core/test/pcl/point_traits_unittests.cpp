#include "sem/core/pcl/point_traits.h"

#ifdef __WITH_PCL // the following tests require PCL support

#include "sem/ts/pcl_point_typed_tests.h"
#include "sem/ts/ts.h"

using namespace pcl;
using namespace sem;
using namespace sem::ts;

namespace {

template<typename TPoint>
class PCL_Point_Traits_TypedTests : public ::testing::Test
{
protected:
};

TYPED_TEST_CASE(PCL_Point_Traits_TypedTests, PCLPointTypes);

TYPED_TEST(PCL_Point_Traits_TypedTests, FieldCount)
{
    ASSERT_GT(PCLPointTraits_<TypeParam >::FieldCount(), size_t(0)) << "This point type does not have any fields.";
    EXPECT_EQ(PCLPointTraits_<TypeParam >::FieldCount(), ExpectedPointAttr_<TypeParam >::field_count);
}

TYPED_TEST(PCL_Point_Traits_TypedTests, NbFloats)
{
    ASSERT_GT(PCLPointTraits_<TypeParam >::NbFloats()*sizeof(float), size_t(0));
    EXPECT_EQ(sizeof(TypeParam), PCLPointTraits_<TypeParam >::NbFloats()*sizeof(float));
    EXPECT_EQ(PCLPointTraits_<TypeParam >::NbFloats(), ExpectedPointAttr_<TypeParam >::nb_floats);
}

} // annonymous namespace for unit tests

#endif // __WITH_PCL


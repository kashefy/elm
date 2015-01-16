#include "core/pcl/cloud_.h"

#include "ts/ts.h"

using namespace std;
using namespace cv;
using namespace pcl;
using namespace sem;

namespace {

template<typename TPoint>
class PCL_Cloud_T_TypedTests : public ::testing::Test
{
protected:
};
typedef ::testing::Types<PointXYZ, Normal, PointNormal> PCLPointTypes;

/** @brief helper struct with expected results
 */
template <typename TPoint>
struct Expected
{
    static const size_t field_count;
};

template<> const size_t Expected<PointXYZ>::field_count     = 3;    // x, y, z
template<> const size_t Expected<Normal>::field_count       = 4;    // n1, n2, n3, curvature
template<> const size_t Expected<PointNormal>::field_count  = 7;    // PointXYZ + Normal = 4+3

TYPED_TEST_CASE(PCL_Cloud_T_TypedTests, PCLPointTypes);

TYPED_TEST(PCL_Cloud_T_TypedTests, FieldCount)
{
    ASSERT_GT(FieldCount<TypeParam >(), size_t(0)) << "This point type does not have any fields.";
    EXPECT_EQ(FieldCount<TypeParam >(), Expected<TypeParam >::field_count);
}

TYPED_TEST(PCL_Cloud_T_TypedTests, NbFloats)
{
    ASSERT_GT(NbFloats<TypeParam >()*sizeof(float), size_t(0));
    EXPECT_EQ(sizeof(TypeParam), NbFloats<TypeParam >()*sizeof(float));
}


} // annonymous namespace for tests

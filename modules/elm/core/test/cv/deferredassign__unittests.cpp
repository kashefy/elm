#include "elm/core/cv/deferredassign_.h"

#include "elm/ts/layer_assertions.h"

using namespace cv;
using namespace elm;

namespace {

/**
 * @brief Typed tests around DeferredAssign_ class POD types
 */
template <class T>
class DeferredAssign_TypedTest : public ::testing::Test
{
protected:
};
typedef ::testing::Types<float, int> PODTypes;
TYPED_TEST_CASE(DeferredAssign_TypedTest, PODTypes);

TYPED_TEST(DeferredAssign_TypedTest, Assign_empty_subs)
{
    {
        Mat_<TypeParam > m0 = Mat_<TypeParam >::ones(2, 3);
        Mat_<TypeParam > m = m0.clone();
        DeferredAssign_<TypeParam > to;
        to.assign(m);
        EXPECT_MAT_EQ(m0, m);
    }
    {
        Mat_<TypeParam > m0 = Mat_<TypeParam >::zeros(2, 3);
        Mat_<TypeParam > m = m0.clone();
        DeferredAssign_<TypeParam > to;
        to.assign(m);
        EXPECT_MAT_EQ(m0, m);
    }
}

TYPED_TEST(DeferredAssign_TypedTest, Assign_empty_subs_empty_mat)
{
    DeferredAssign_<TypeParam > to;
    Mat_<TypeParam > m;
    EXPECT_NO_THROW(to.assign(m));
}

} // annonymous namespace for tests

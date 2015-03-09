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

TYPED_TEST(DeferredAssign_TypedTest, Assign)
{
    const int N = 6;
    float data[N] = {1.f, 2.f,
                     1.f, 4.f,
                     6.f, 1.f};
    Mat_<TypeParam > m = Mat1f(3, 2, data).clone();

    DeferredAssign_<TypeParam > to;

    to.push_back(1, 2);
    to.push_back(4, 9);

    to.assign(m);

    float data2[N] = {2.f, 2.f,
                      2.f, 9.f,
                      6.f, 2.f};

    Mat_<TypeParam > expected = Mat1f(3, 2, data2).clone();
    EXPECT_MAT_EQ(expected, m);
}

TYPED_TEST(DeferredAssign_TypedTest, Assign_chained)
{
    const int N = 6;
    float data[N] = {1.f, 2.f,
                     1.f, 4.f,
                     6.f, 1.f};
    Mat_<TypeParam > m = Mat1f(3, 2, data).clone();

    DeferredAssign_<TypeParam > to;

    to.push_back(1, 2);
    to.push_back(4, 9);
    to.push_back(2, 9);

    to.assign(m);

    float data2[N] = {9.f, 9.f,
                      9.f, 9.f,
                      6.f, 9.f};

    Mat_<TypeParam > expected = Mat1f(3, 2, data2).clone();
    EXPECT_MAT_EQ(expected, m);
}

TYPED_TEST(DeferredAssign_TypedTest, Assign_chained_no_repeat)
{
    const int N = 6;
    float data[N] = {1.f, 2.f,
                     1.f, 4.f,
                     6.f, 1.f};
    Mat_<TypeParam > m = Mat1f(3, 2, data).clone();
    Mat_<TypeParam > m0 = m.clone();
    Mat_<TypeParam > m01 = m.clone();

    DeferredAssign_<TypeParam > to;

    to.push_back(1, 2);
    to.push_back(4, 9);
    to.push_back(2, 9);

    to.assign(m);

    float data2[N] = {9.f, 9.f,
                      9.f, 9.f,
                      6.f, 9.f};

    Mat_<TypeParam > expected = Mat1f(3, 2, data2).clone();
    EXPECT_MAT_EQ(expected, m);

    // second time yields no changes
    to.assign(m);
    EXPECT_MAT_EQ(expected, m);

    to.assign(m0);
    EXPECT_MAT_EQ(m01, m0);
}


TYPED_TEST(DeferredAssign_TypedTest, Assign_chained_empty_mat)
{
    Mat_<TypeParam > m = Mat1f();

    DeferredAssign_<TypeParam > to;

    to.push_back(1, 2);
    to.push_back(4, 9);
    to.push_back(2, 9);

    to.assign(m);
}

TYPED_TEST(DeferredAssign_TypedTest, Clear)
{
    const int N = 6;
    float data[N] = {1.f, 2.f,
                     1.f, 4.f,
                     6.f, 1.f};
    Mat_<TypeParam > m = Mat1f(3, 2, data).clone();
    Mat_<TypeParam > m0 = m.clone();

    DeferredAssign_<TypeParam > to;

    to.push_back(1, 2);
    to.push_back(4, 9);
    to.push_back(2, 9);

    to.clear();
    to.assign(m);

    // second time yields no changes
    to.assign(m);
    EXPECT_MAT_EQ(m0, m);
}

} // annonymous namespace for tests

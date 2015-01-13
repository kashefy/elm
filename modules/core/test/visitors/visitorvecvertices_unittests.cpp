/** @file test out VecVertices visitor
 */
#ifdef __WITH_PCL // PCL support required for these tests

#include "core/visitors/visitorvertices.h"

#include "core/exception.h"
#include "core/pcl/cloud.h"
#include "ts/ts.h"

using namespace std;
using namespace cv;
using namespace sem;

namespace {

/**
 * @brief test class around VisitorVecVertices
 */
class VisitorVecVerticesTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        to_.Reset();
    }

    VisitorVecVertices to_;    ///< test object
};

TEST_F(VisitorVecVerticesTest, EmptyVecVertices)
{
    EXPECT_TRUE(to_(VecVertices()).empty());
}

TEST_F(VisitorVecVerticesTest, EmptyVecVertices_Size)
{
    EXPECT_SIZE(0, to_(VecVertices()));
}

TEST_F(VisitorVecVerticesTest, Twos_cloud)
{
    CloudXYZPtr in = Mat2PointCloud(Mat1f(4, 3, 2));

    VecVertices vv = to_(in);

    EXPECT_SIZE(4, vv);
    EXPECT_SIZE(4, vv[0].vertices);

    for(uint32_t i=0; i<in->height; i++) {
        for(uint32_t j=0; j<in->width; j++) {

            EXPECT_EQ(static_cast<uint32_t>(2), vv[i].vertices[j]);
        }
    }
}

template<class T>
class VisitorVecVerticesTypedTest : public VisitorVecVerticesTest
{
protected:
    virtual void SetUp()
    {
        to_.Reset();
    }

    //typedef std::list<T> List;
     static T shared_;
     T value_;
};

/**
 * @brief Value per type to use in fixtures below.
 */
template<class T>
struct Twos_
{
    static const T val;
};

///< Register static variables to work with inside tests
template<> const float Twos_<float>::val    = 2.f;
template<> const int Twos_<int>::val        = 2;
template<> const uchar Twos_<uchar>::val    = 2;
template<> const Mat1f Twos_<Mat1f>::val    = Mat1f(1, 1, 2.f);

typedef ::testing::Types<Mat1f, float, int, uchar> VisitorTypes;
TYPED_TEST_CASE(VisitorVecVerticesTypedTest, VisitorTypes);

TYPED_TEST(VisitorVecVerticesTypedTest, Twos)
{
    TypeParam in = Twos_<TypeParam >::val;

    VecVertices vv = to_(in);

    EXPECT_SIZE(1, vv);
    EXPECT_SIZE(1, vv[0].vertices);

    EXPECT_EQ(static_cast<uint32_t>(2), vv[0].vertices[0]);
}

} // annonymous namespace for VecVertices visitors' test fixtures

#else // __WITH_PCL
    #warning "Skipping building VecVertices visitor unit tests due to no pcl support."
#endif // __WITH_PCL

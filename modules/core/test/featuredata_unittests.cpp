#include "core/featuredata.h"

#include "core/core.h"
#include "ts/ts.h"

using namespace std;
using namespace cv;
using namespace sem;

namespace {

/**
 * @brief test class around FeatDataVisitorMatFTest
 */
class FeatDataVisitorMatFTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
    }

    FeatDataVisitorMatF to_;    ///< test object
};

TEST_F(FeatDataVisitorMatFTest, Empty)
{
    EXPECT_TRUE(to_(Mat_f()).empty());
    EXPECT_TRUE(to_(Mat_f(0, 0)).empty());
    EXPECT_TRUE(to_(Mat_f(1, 0)).empty());
    EXPECT_TRUE(to_(Mat_f(0, 1)).empty());
}


TEST_F(FeatDataVisitorMatFTest, EmptySize)
{
    EXPECT_MAT_DIMS_EQ(to_(Mat_f()), Size2i(0, 0));
    EXPECT_MAT_DIMS_EQ(to_(Mat_f(0, 0)), Size2i(0, 0));
    EXPECT_MAT_DIMS_EQ(to_(Mat_f(1, 0)), Size2i(0, 1));
    EXPECT_MAT_DIMS_EQ(to_(Mat_f(0, 1)), Size2i(1, 0));
}

TEST_F(FeatDataVisitorMatFTest, MatF)
{
    int N=5;

    while(N--) {

        const int MAT_ROWS = abs(randu<int>()) % 10 + 1;
        const int MAT_COLS = abs(randu<int>()) % 100 + 1;

        Mat_f m(MAT_ROWS, MAT_COLS);
        randn(m, 0.f, 100.f);

        Mat_f m2 = to_(m);

        EXPECT_MAT_EQ(m, m2) << "Matrices are not equal";
        EXPECT_EQ(m.data, m2.data) << "Mats are not pointing to the same data in memory. Expecting shared copy.";
    }
}

TEST_F(FeatDataVisitorMatFTest, Cloud)
{
    Mat1f m(4, 3);
    randn(m, 0.f, 100.f);
    CloudXYZ::Ptr cloud = Mat2PointCloud(m);

    Mat_f m2 = to_(cloud);
    hconcat(m, Mat1f(m.rows, 1, 1), m);

    EXPECT_MAT_EQ(m, m2);
    EXPECT_NE(m.data, m2.data) << "Expecting deep copy. If intentionally optimized to be a shared copy, please update test.";
}

class FeatureDataTest : public ::testing::Test
{

};

TEST_F(FeatureDataTest, Foo)
{
    Mat_f m(1, 3, 2);
    FeatureData x(m);
    CloudXYZ::Ptr cld = x.get<CloudXYZ::Ptr>();


    cld = Mat2PointCloud(m);
    FeatureData y(cld);

    Mat_f m2 = y.get<Mat_f >();
}

} // annonymous namespace for test fixtures

/** @file test out cloud visitor
 */
#ifdef __WITH_PCL // PCL support required for these tests
#include "sem/core/visitors/visitorcloud.h"

#include "sem/core/exception.h"
#include "sem/core/pcl/vertices.h"
#include "sem/ts/ts.h"

using namespace std;
using namespace pcl;
using namespace cv;
using namespace sem;

namespace {

/**
 * @brief test class around VisitorCloud
 */
class VisitorCloudTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        to_.Reset();
    }

    VisitorCloud to_;    ///< test object
};

TEST_F(VisitorCloudTest, Empty)
{
    EXPECT_TRUE(to_(Mat_f())->empty());
    EXPECT_TRUE(to_(Mat_f(0, 0))->empty());
    EXPECT_TRUE(to_(Mat_f(1, 0))->empty());
    EXPECT_TRUE(to_(Mat_f(0, 1))->empty());

    CloudXYZPtr cld(new CloudXYZ);
    EXPECT_TRUE(to_(cld)->empty());
}

TEST_F(VisitorCloudTest, EmptySize)
{
    EXPECT_EQ(size_t(0), to_(Mat_f())->size());
    EXPECT_EQ(size_t(0), to_(Mat_f(0, 0))->size());
    EXPECT_EQ(size_t(0), to_(Mat_f(1, 0))->size());
    EXPECT_EQ(size_t(0), to_(Mat_f(0, 1))->size());

    CloudXYZPtr cld(new CloudXYZ);
    EXPECT_EQ(size_t(0), to_(cld)->size());
}

TEST_F(VisitorCloudTest, FromMat_f)
{
    Mat1f m(4, 3);
    randn(m, 0.f, 100.f);

    CloudXYZPtr cld = to_(m);
    Mat1f m2 = PointCloud2Mat_<PointXYZ>(cld);
    hconcat(m, Mat1f(m.rows, 1, 1), m);

    EXPECT_MAT_EQ(m, m2);
    EXPECT_NE(m.data, m2.data) << "Expecting deep copy. If intentionally optimized to be a shared copy, please update test.";
}

TEST_F(VisitorCloudTest, Cloud)
{
    Mat1f m(4, 3);
    randn(m, 0.f, 100.f);
    CloudXYZPtr cld = Mat2PointCloud_<PointXYZ>(m);

    CloudXYZPtr cld2 = to_(cld);

    EXPECT_EQ(cld->size(), cld2->size()) << "Size mismatch.";
    EXPECT_EQ(cld->width, cld2->width) << "Width mismatch.";
    EXPECT_EQ(cld->height, cld2->height) << "Height mismatch.";

    for(CloudXYZ::iterator itr1=cld->begin(), itr2=cld2->begin();
        itr1 != cld->end(); ++itr1, ++itr2) {

        pcl::PointXYZ _p1 = *itr1;
        pcl::PointXYZ _p2 = *itr2;
        EXPECT_FLOAT_EQ(_p1.x, _p2.x) << "Unexpected value for x coordinate.";
        EXPECT_FLOAT_EQ(_p1.y, _p2.y) << "Unexpected value for y coordinate.";
        EXPECT_FLOAT_EQ(_p1.z, _p2.z) << "Unexpected value for z coordinate.";
    }
}

TEST_F(VisitorCloudTest, ResetValid)
{
    Mat1f m(4, 3);
    randn(m, 0.f, 100.f);

    CloudXYZPtr cld1 = to_(m);
    EXPECT_NO_THROW(to_.Reset());

    CloudXYZPtr cld2 = Mat2PointCloud_<PointXYZ>(m);
    CloudXYZPtr cld3 = to_(cld2);
    EXPECT_NO_THROW(to_.Reset());

    cld1 = to_(m);
    EXPECT_NO_THROW(to_.Reset());
}

/**
 * @brief Thest this visitor's caching
 */
TEST_F(VisitorCloudTest, Reset)
{
    Mat1f m(4, 3, 1.f);
    Mat1f m0 = m.clone();
    hconcat(m0, Mat1f(m0.rows, 1, 1), m0);

    CloudXYZPtr cld = to_(m);
    EXPECT_MAT_EQ(PointCloud2Mat_<PointXYZ>(cld), m0);

    cld = to_(m); // result of first call is cached.

    const int N=2;
    ASSERT_GT(N, 1) << "This test must iterate at least twice to see effect of Reset() on cached reference.";
    for(int i=0; i<2; i++) {

        cld = to_(m+1);

        if(i == 0) {

            EXPECT_MAT_EQ(PointCloud2Mat_<PointXYZ>(cld), m0) << "Not suing cahced cloud.";
        }
        else {

            Mat1f m2 = m0.clone();
            m2.colRange(0, m2.cols-1).setTo(2.f);
            EXPECT_MAT_EQ(PointCloud2Mat_<PointXYZ>(cld), m2) << "Still using cached cloud";
        }
        to_.Reset();
    }
}

TEST_F(VisitorCloudTest, FromVecVertices)
{
    Mat1f m(4, 3);
    for(size_t i=0; i<m.total(); i++) {

        m(i) = static_cast<float>(randu<uint32_t>() % 256);
    }

    CloudXYZPtr cld = to_(Mat2VecVertices(m));

    Mat1f m2 = PointCloud2Mat_<PointXYZ>(cld);
    hconcat(m, Mat1f(m.rows, 1, 1), m);

    EXPECT_MAT_EQ(m, m2);
    EXPECT_NE(m.data, m2.data) << "Expecting deep copy. If intentionally optimized to be a shared copy, please update test.";
}

} // annonymous namespace for cloud visitors' test fixtures

#else // __WITH_PCL
    #warning "Skipping building cloud visitor unit tests due to no pcl support."
#endif // __WITH_PCL

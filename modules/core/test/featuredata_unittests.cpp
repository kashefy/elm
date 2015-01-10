#include "core/featuredata.h"

#include "core/exception.h"
#include "ts/ts.h"

using namespace std;
using namespace cv;
using namespace sem;

namespace {

class FeatureDataTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        mat_ = Mat_f(4, 3);
        randn(mat_, 0.f, 100.f);

        cld_ = Mat2PointCloud(mat_.clone());
        mat_ = PointCloud2Mat(cld_).clone();
    }

    Mat_f mat_;
    CloudXYZ::Ptr cld_;
};

/**
 * @brief Initialize test object with Cloud object, then call getters.
 */
TEST_F(FeatureDataTest, Init_Cloud)
{
    FeatureData to(cld_);

    Mat_f m = to.get<Mat_f>();
    CloudXYZ::Ptr cld = to.get<CloudXYZ::Ptr>();

    EXPECT_MAT_EQ(m, mat_);
    EXPECT_MAT_EQ(PointCloud2Mat(cld), mat_);
}

/**
 * @brief Initialize test object with Mat object, then call getters.
 */
TEST_F(FeatureDataTest, Init_Mat_f)
{
    FeatureData to(mat_);

    Mat_f m = to.get<Mat_f>();
    CloudXYZ::Ptr cld = to.get<CloudXYZ::Ptr>();

    EXPECT_MAT_EQ(m, mat_);
    EXPECT_MAT_EQ(PointCloud2Mat(cld), mat_);
}

/**
 * @brief Verify initializing with Mat and getting a mat only involves a shared copy
 */
TEST_F(FeatureDataTest, Same_Mat)
{
    FeatureData to(mat_);

    Mat_f m = to.get<Mat_f>();
    EXPECT_MAT_EQ(m, mat_);
    EXPECT_EQ(m.data, mat_.data) << "Both Mats are not pointing to the same memory location.";
    ASSERT_NE(m.clone().data, mat_.data) << "Cannot still be pointing to the same memory chunk after cloning.";
}

/**
 * @brief Test caching of cloud reference
 * by comparing what the pointers are pointing at and verifying the use count increased by 1 after calling get
 */
TEST_F(FeatureDataTest, Cached_Cloud)
{
    FeatureData to(mat_);

    CloudXYZ::Ptr cld = to.get<CloudXYZ::Ptr>();
    EXPECT_NE(cld, cld_);
    long old_cld_use_count = cld.use_count();

    CloudXYZ::Ptr cld2 = to.get<CloudXYZ::Ptr>();
    EXPECT_NE(cld2, cld_);
    EXPECT_EQ(cld2, cld);

    // check use count increased
    EXPECT_EQ(cld2.use_count(), cld.use_count());
    EXPECT_EQ(cld2.use_count(), old_cld_use_count+1);
}

} // annonymous namespace for test fixtures

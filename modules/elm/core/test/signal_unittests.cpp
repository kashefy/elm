/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/core/signal.h"

#include "elm/core/featuredata.h"
#include "elm/core/exception.h"
#include "elm/ts/signal_tp_.h"
#include "elm/ts/ts.h"

using namespace std;
using namespace cv;
using namespace elm;

INSTANTIATE_TYPED_TEST_CASE_P(SignalTP_FtData_Tests, Signal_TP_, FeatureData);

template<>
std::vector<FeatureData> FtV_<FeatureData>::values{FeatureData(Mat1f(1, 1, 0.f)),
            FeatureData(Mat1f(1, 1, 1.f)),
            FeatureData(Mat1f(1, 1, 100.f))};

/**
 * @brief Additional coverage for main Signal class with FeatureData template
 */
class SignalTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        to_ = Signal();

        VecS in;
        in.push_back("foo");
        in.push_back("bar");

        for(size_t i=0; i<in.size(); i++) {

            to_.Append(in[i], Mat1f());
        }

        to_.Append(in[0], Mat1f());
    }

    Signal to_; ///<test object
    VecS in_;   ///< test feature names
};

TEST_F(SignalTest, GetFeatures)
{
    EXPECT_THROW(to_["wrong"], ExceptionKeyError);
    EXPECT_THROW(to_["Foo"], ExceptionKeyError);
    EXPECT_THROW(to_["FOO"], ExceptionKeyError);

    EXPECT_SIZE(2, to_["foo"]);
    EXPECT_SIZE(1, to_["bar"]);

    EXPECT_MAT_EQ(to_["bar"][0], Mat1f());
}

TEST_F(SignalTest, MostRecentMat1f)
{
    EXPECT_THROW(to_.MostRecentMat1f("wrong"), ExceptionKeyError);
    EXPECT_THROW(to_.MostRecentMat1f("Foo"), ExceptionKeyError);
    EXPECT_THROW(to_.MostRecentMat1f("FOO"), ExceptionKeyError);

    for(int i=0; i<3; i++) {

        EXPECT_SIZE(2+i, to_["foo"]);
        to_.Append("foo", Mat1f(1, 1, i+1));
        EXPECT_SIZE(2+i+1, to_["foo"]);
        EXPECT_MAT_EQ(to_.MostRecentMat1f("foo"), Mat1f::ones(1, 1)+i);
    }
}

TEST_F(SignalTest, AppendMat1f)
{
    Signal to; ///< test object

    to.Append("foo", Mat1f(3, 2, 1.f));
    EXPECT_SIZE(1, to.FeatureNames());
    EXPECT_MAT_EQ(Mat1f(3, 2, 1.f), to.MostRecentMat1f("foo"));

    to.Append("foo", Mat1f(3, 2, 2.f));
    EXPECT_SIZE(1, to.FeatureNames());
    EXPECT_MAT_EQ(Mat1f(3, 2, 2.f), to.MostRecentMat1f("foo"));

    to.Append("bar", Mat1f(3, 2, 3.f));
    EXPECT_SIZE(2, to.FeatureNames());
    EXPECT_MAT_EQ(Mat1f(3, 2, 3.f), to.MostRecentMat1f("bar"));
    EXPECT_MAT_EQ(Mat1f(3, 2, 2.f), to.MostRecentMat1f("foo"));
}

TEST_F(SignalTest, AppendFeatureData)
{
    Signal to; ///< test object

    to.Append("foo", FeatureData(Mat1f(3, 2, 1.f)));
    EXPECT_SIZE(1, to.FeatureNames());
    EXPECT_MAT_EQ(Mat1f(3, 2, 1.f), to.MostRecentMat1f("foo"));

    to.Append("foo", FeatureData(Mat1f(3, 2, 2.f)));
    EXPECT_SIZE(1, to.FeatureNames());
    EXPECT_MAT_EQ(Mat1f(3, 2, 2.f), to.MostRecentMat1f("foo"));

    to.Append("bar", FeatureData(Mat1f(3, 2, 3.f)));
    EXPECT_SIZE(2, to.FeatureNames());
    EXPECT_MAT_EQ(Mat1f(3, 2, 3.f), to.MostRecentMat1f("bar"));
    EXPECT_MAT_EQ(Mat1f(3, 2, 2.f), to.MostRecentMat1f("foo"));
}

TEST_F(SignalTest, AppendSparseMat1f)
{
    Signal to; ///< test object

    to.Append("foo", SparseMat1f(Mat1f(3, 2, 1.f)));
    EXPECT_SIZE(1, to.FeatureNames());
    EXPECT_MAT_EQ(Mat1f(3, 2, 1.f), to.MostRecentMat1f("foo"));

    to.Append("foo", SparseMat1f(Mat1f(3, 2, 2.f)));
    EXPECT_SIZE(1, to.FeatureNames());
    EXPECT_MAT_EQ(Mat1f(3, 2, 2.f), to.MostRecentMat1f("foo"));

    to.Append("bar", SparseMat1f(Mat1f(3, 2, 3.f)));
    EXPECT_SIZE(2, to.FeatureNames());
    EXPECT_MAT_EQ(Mat1f(3, 2, 3.f), to.MostRecentMat1f("bar"));
    EXPECT_MAT_EQ(Mat1f(3, 2, 2.f), to.MostRecentMat1f("foo"));
}

TEST_F(SignalTest, AppendVecMat1f)
{
    Signal to; ///< test object

    {
        VecMat1f v;
        v.push_back(Mat1f(1, 1, 1.f));
        to.Append("foo", v);
        EXPECT_SIZE(1, to.FeatureNames());

        VecMat1f v2 = to.MostRecent("foo").get<VecMat1f>();
        EXPECT_SIZE(1, v2);
        EXPECT_MAT_EQ(Mat1f(1, 1, 1.f), v2[0]);
    }

    {
        VecMat1f v;
        v.push_back(Mat1f(2, 1, 1.f));
        v.push_back(Mat1f(2, 2, 2.f));
        to.Append("foo", v);
        EXPECT_SIZE(1, to.FeatureNames());
        VecMat1f v2 = to.MostRecent("foo").get<VecMat1f>();
        EXPECT_SIZE(2, v2);
        EXPECT_MAT_EQ(Mat1f(2, 1, 1.f), v2[0]);
        EXPECT_MAT_EQ(Mat1f(2, 2, 2.f), v2[1]);
    }

    {
        VecMat1f v;
        v.push_back(Mat1f(3, 1, 1.f));
        v.push_back(Mat1f(3, 2, 2.f));
        v.push_back(Mat1f(3, 3, 3.f));
        to.Append("bar", v);
        EXPECT_SIZE(2, to.FeatureNames());

        VecMat1f v2 = to.MostRecent("bar").get<VecMat1f>();
        EXPECT_SIZE(3, v2);
        EXPECT_MAT_EQ(Mat1f(3, 1, 1.f), v2[0]);
        EXPECT_MAT_EQ(Mat1f(3, 2, 2.f), v2[1]);
        EXPECT_MAT_EQ(Mat1f(3, 3, 3.f), v2[2]);

        v2 = to.MostRecent("foo").get<VecMat1f>();
        EXPECT_SIZE(2, v2);
        EXPECT_MAT_EQ(Mat1f(2, 1, 1.f), v2[0]);
        EXPECT_MAT_EQ(Mat1f(2, 2, 2.f), v2[1]);
    }
}

#ifdef __WITH_PCL

TEST_F(SignalTest, AppendCloudXYZ)
{
    Signal to; ///< test object

    to.Append("foo", Mat2PointCloud_<pcl::PointXYZ >(Mat1f(1, 3, 1.f)));
    EXPECT_SIZE(1, to.FeatureNames());
    EXPECT_MAT_EQ(Mat1f(1, 4, 1.f), to.MostRecentMat1f("foo"));

    to.Append("foo", Mat2PointCloud_<pcl::PointXYZ > (Mat1f(2, 3, 1.f)));
    EXPECT_SIZE(1, to.FeatureNames());
    EXPECT_MAT_EQ(Mat1f(2, 4, 1.f), to.MostRecentMat1f("foo"));

    to.Append("bar", Mat2PointCloud_<pcl::PointXYZ > (Mat1f(3, 3, 1.f)));
    EXPECT_SIZE(2, to.FeatureNames());
    EXPECT_MAT_EQ(Mat1f(3, 4, 1.f), to.MostRecentMat1f("bar"));
    EXPECT_MAT_EQ(Mat1f(2, 4, 1.f), to.MostRecentMat1f("foo"));
}

#endif // __WITH_PCL

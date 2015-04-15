/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/layers/icp.h"

#ifdef __WITH_PCL // test normally

#include <pcl/registration/registration.h>
#include <pcl/registration/icp.h>

#include <Eigen/Dense>            // to enable OpenCV's eigen2cv()
#include <opencv2/core/eigen.hpp> // for eigen2cv(), preceeded be #include <Eigen/Dense>

#include "elm/core/exception.h"
#include "elm/core/layerconfig.h"
#include "elm/core/pcl/cloud_.h"
#include "elm/core/signal.h"
#include "elm/core/stl/stl_inl.h"
#include "elm/layers/layerfactory.h"
#include "elm/ts/layer_assertions.h"

using namespace std;
using namespace cv;
using namespace pcl;
using namespace elm;

namespace {

ELM_INSTANTIATE_LAYER_TYPED_TEST_CASE_P(ICP);

const string NAME_INPUT_POINT_CLOUD_SRC     = "src";    ///< key to source cloud
const string NAME_INPUT_POINT_CLOUD_TARGET  = "target"; ///< key to target cloud

const string NAME_OUTPUT_SCORE              = "score";  ///< key to fitness score
const string NAME_OUTPUT_CONVERGENCE        = "c";      ///< key to convergence result
const string NAME_OUTPUT_TRANSFORMATION     = "transf"; ///< key to optional final tansformation

class ICPInitTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        cfg_ = LayerConfig();
        cfg_.Params(params_);
        io_names_ = LayerIONames();

        io_names_.Input(ICP::KEY_INPUT_POINT_CLOUD_SRC   , NAME_INPUT_POINT_CLOUD_SRC);
        io_names_.Input(ICP::KEY_INPUT_POINT_CLOUD_TARGET, NAME_INPUT_POINT_CLOUD_TARGET);
        io_names_.Output(ICP::KEY_OUTPUT_CONVERGENCE     , NAME_OUTPUT_CONVERGENCE);
        io_names_.Output(ICP::KEY_OUTPUT_SCORE           , NAME_OUTPUT_SCORE);
        io_names_.Output(ICP::KEY_OUTPUT_TRANSFORMATION  , NAME_OUTPUT_TRANSFORMATION);
    }

    virtual void TearDown()
    {
        params_.clear();
    }

    // members
    PTree params_;
    LayerConfig cfg_;
    LayerIONames io_names_;
};

TEST_F(ICPInitTest, Constructor)
{
    EXPECT_NO_THROW(ICP to(cfg_));
}

TEST_F(ICPInitTest, MissingParams)
{
    cfg_.Params(PTree());
    EXPECT_NO_THROW(ICP to(cfg_));

    ICP to;
    EXPECT_NO_THROW(to.Reset(cfg_));
    EXPECT_NO_THROW(to.Reconfigure(cfg_));
}

TEST_F(ICPInitTest, CreateWithFactory)
{
    shared_ptr<base_Layer> to_ptr = LayerFactory::CreateShared("ICP", cfg_, io_names_);

    EXPECT_TRUE(bool(to_ptr));
}

/**
 * @brief Test the live methods assuming a sucessful initialization
 */
class ICPTest : public ICPInitTest
{
protected:
    virtual void SetUp()
    {
        ICPInitTest::SetUp();

        to_ = LayerFactory::CreateShared("ICP", cfg_, io_names_);

        cloud_in_.reset(new CloudXYZ);

        // Fill in the CloudIn data
        cloud_in_->width    = 5;
        cloud_in_->height   = 1;
        cloud_in_->is_dense = false;
        cloud_in_->points.resize(cloud_in_->width * cloud_in_->height);
        for(size_t i=0; i < cloud_in_->points.size(); ++i) {

            cloud_in_->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
            cloud_in_->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
            cloud_in_->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
        }

        // target.x = in.x + constant
        cloud_target_.reset(new CloudXYZ);
        *cloud_target_ = *cloud_in_;
        for(size_t i=0; i < cloud_in_->points.size (); ++i) {

            cloud_target_->points[i].x += 0.7f;
        }

        sig_.Append(NAME_INPUT_POINT_CLOUD_SRC, PointCloud2Mat_<PointXYZ>(cloud_in_));
        sig_.Append(NAME_INPUT_POINT_CLOUD_TARGET, PointCloud2Mat_<PointXYZ>(cloud_target_));
    }

    virtual void TearDown()
    {
        ICPInitTest::TearDown();
    }

    shared_ptr<base_Layer> to_; ///< pointer to test object
    Signal sig_;
    CloudXYZPtr cloud_in_;
    CloudXYZPtr cloud_target_;
};

/**
 * @brief Compare Layer ICP with PCL's ICP routines
 */
TEST_F(ICPTest, ActivateAndResponse) {

    IterativeClosestPoint<PointXYZ, PointXYZ> icp;
    icp.setInputSource(cloud_in_);
    icp.setInputTarget(cloud_target_);

    CloudXYZ fin;
    icp.align(fin);

    to_->Activate(sig_);

    EXPECT_FALSE(sig_.Exists(NAME_OUTPUT_CONVERGENCE));
    EXPECT_FALSE(sig_.Exists(NAME_OUTPUT_SCORE));
    EXPECT_FALSE(sig_.Exists(NAME_OUTPUT_TRANSFORMATION));

    to_->Response(sig_);

    EXPECT_TRUE(sig_.Exists(NAME_OUTPUT_CONVERGENCE));
    EXPECT_TRUE(sig_.Exists(NAME_OUTPUT_SCORE));
    EXPECT_TRUE(sig_.Exists(NAME_OUTPUT_TRANSFORMATION));

    // check response
    // convergence:
    ASSERT_TRUE(icp.hasConverged()) << "this test needs to converge";
    EXPECT_MAT_DIMS_EQ(sig_.MostRecentMat1f(NAME_OUTPUT_CONVERGENCE), Size2i(1, 1)) << "Expecting Mat of single element for convergence.";
    EXPECT_EQ(sig_.MostRecentMat1f(NAME_OUTPUT_CONVERGENCE).at<float>(0)==1.f, icp.hasConverged()) << "Mismatched convergence.";

    // fitness score
    EXPECT_MAT_DIMS_EQ(sig_.MostRecentMat1f(NAME_OUTPUT_SCORE), Size2i(1, 1)) << "Expecting Mat of single element for fitness score.";
    EXPECT_FLOAT_EQ(sig_.MostRecentMat1f(NAME_OUTPUT_SCORE).at<float>(0), static_cast<float>(icp.getFitnessScore())) << "Mismatched fitness score.";

    // final transformation matrix estimated by registration method
    Mat1f transformation_expected;
    eigen2cv(icp.getFinalTransformation(), transformation_expected);

    Mat1f transformation_actual = sig_.MostRecentMat1f(NAME_OUTPUT_TRANSFORMATION);
    EXPECT_MAT_EQ(transformation_expected, transformation_actual) << "Mismatch in final transformation";

    // more detailed check of transformation matrix
    EXPECT_EQ(transformation_actual.rows, transformation_actual.cols) << "Expecting square matrix";
    for(int i=0; i<transformation_actual.rows; i++) {

        EXPECT_NEAR(1.f, transformation_actual(i, i), 1e-5) << "Expecting diagonal of 1's";
    }
}

/**
 * @brief Pick a scenario that would not converge
 */
TEST_F(ICPTest, NoConvergence)
{
    Mat1f in = Mat1f::zeros(1, 3);
    Mat1f target = in.clone();

    sig_.Append(NAME_INPUT_POINT_CLOUD_SRC, in);
    sig_.Append(NAME_INPUT_POINT_CLOUD_TARGET, target);

    to_->Activate(sig_);
    to_->Response(sig_);

    EXPECT_TRUE(sig_.Exists(NAME_OUTPUT_CONVERGENCE));
    EXPECT_TRUE(sig_.Exists(NAME_OUTPUT_SCORE));
    EXPECT_TRUE(sig_.Exists(NAME_OUTPUT_TRANSFORMATION));

    // check response
    // convergence:
    EXPECT_FALSE(sig_.MostRecentMat1f(NAME_OUTPUT_CONVERGENCE).at<float>(0)==1.f) << "Not expecting convergence.";

    // fitness score
    EXPECT_MAT_EQ(sig_.MostRecentMat1f(NAME_OUTPUT_SCORE), Mat1f(1, 1, 0.f)) << "Expecting zero score.";

    // more detailed check of transformation matrix, expecting a diagonal matrix
    Mat1f diagonal(4, 4, 0.f);
    for(int i=0; i<diagonal.rows; i++) {

        diagonal(i, i) = 1.f;
    }

    EXPECT_MAT_EQ(sig_.MostRecentMat1f(NAME_OUTPUT_TRANSFORMATION), diagonal) << "Expecting diagonal matrix when convergence fails.";
}

} // annonymous namespace for test fixtures

#endif // __WITH_PCL

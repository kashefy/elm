#include "layers/icp.h"

#include "pcl/registration/registration.h"
#include "pcl/registration/icp.h"

#include "core/exception.h"
#include "core/layerconfig.h"
#include "core/pcl_utils.h"
#include "core/signal.h"
#include "core/stl.h"
#include "layers/layerfactory.h"
#include "ts/ts.h"
#include "ts/layer_assertions.h"

using namespace std;
using namespace cv;
using namespace pcl;
using namespace sem;

namespace {

#ifdef __WITH_PCL // test normally

const string NAME_INPUT_POINT_CLOUD_SRC;         ///< key to source cloud
const string NAME_INPUT_POINT_CLOUD_TARGET;      ///< key to target cloud

const string NAME_OUTPUT_SCORE;                  ///< key to fitness score
const string NAME_OUTPUT_CONVERGENCE;            ///< key to convergence result
const string NAME_OUTPUT_TRANSFORMATION;         ///< key to optional final tansformation

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
    EXPECT_NO_THROW(ICP to);
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

TEST_F(ICPInitTest, MissingRequiredIONames)
{
    shared_ptr<base_Layer> to_ptr(new ICP()); // pointer to test object

    EXPECT_THROW(to_ptr->IONames(LayerIONames()), ExceptionKeyError);

    map<string, pair<bool, string> > io_pairs; // false for input, true for output
    io_pairs[ICP::KEY_INPUT_POINT_CLOUD_SRC    ] = make_pair(0, NAME_INPUT_POINT_CLOUD_SRC);
    io_pairs[ICP::KEY_INPUT_POINT_CLOUD_TARGET ] = make_pair(0, NAME_INPUT_POINT_CLOUD_TARGET);
    io_pairs[ICP::KEY_OUTPUT_CONVERGENCE       ] = make_pair(1, NAME_OUTPUT_CONVERGENCE);
    io_pairs[ICP::KEY_OUTPUT_SCORE             ] = make_pair(1, NAME_OUTPUT_SCORE);
    io_pairs[ICP::KEY_OUTPUT_TRANSFORMATION    ] = make_pair(1, NAME_OUTPUT_TRANSFORMATION);

    ValidateRequiredIONames(io_pairs, to_ptr);
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

        cloud_in_.reset(new PointCloudXYZ);

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
        cloud_target_.reset(new PointCloudXYZ);
        *cloud_target_ = *cloud_in_;
        for(size_t i=0; i < cloud_in_->points.size (); ++i) {

            cloud_target_->points[i].x += 0.7f;
        }

        sig_.Append(NAME_INPUT_POINT_CLOUD_SRC, PointCloud2Mat(cloud_in_));
        sig_.Append(NAME_INPUT_POINT_CLOUD_TARGET, PointCloud2Mat(cloud_target_));
    }

    virtual void TearDown()
    {
        ICPInitTest::TearDown();
    }

    shared_ptr<base_Layer> to_; ///< pointer to test object
    Signal sig_;
    PointCloudXYZ::Ptr cloud_in_;
    PointCloudXYZ::Ptr cloud_target_;
};

TEST_F(ICPTest, Activate) {

    IterativeClosestPoint<PointXYZ, PointXYZ> icp;
    icp.setInputSource(cloud_in_);
    icp.setInputTarget(cloud_target_);
    PointCloudXYZ fin;
    icp.align(fin);
    cout << "has converged:" << icp.hasConverged() << " score: " <<
                 icp.getFitnessScore() << endl;
    cout << icp.getFinalTransformation() << endl;

    EXPECT_FALSE(sig_.Exists(NAME_OUTPUT_CONVERGENCE));
    EXPECT_FALSE(sig_.Exists(NAME_OUTPUT_SCORE));
    EXPECT_FALSE(sig_.Exists(NAME_OUTPUT_TRANSFORMATION));

    to_->Response(sig_);

    EXPECT_TRUE(sig_.Exists(NAME_OUTPUT_CONVERGENCE));
    EXPECT_TRUE(sig_.Exists(NAME_OUTPUT_SCORE));
}

#else // __WITH_PCL
    #warning "Skipping building ICP layer unittests"
#endif // __WITH_PCL

} // annonymous namespace for test fixtures

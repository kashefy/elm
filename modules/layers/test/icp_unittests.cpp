#include "layers/icp.h"

#include "core/exception.h"
#include "core/layerconfig.h"
#include "core/signal.h"
#include "core/stl.h"
#include "layers/layerfactory.h"
#include "ts/ts.h"
#include "ts/layer_assertions.h"

using namespace std;
using namespace cv;
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

/**
 * @brief Test the live methods assuming a sucessful initialization
 */
class ICPTest : public ICPInitTest
{
protected:
    virtual void SetUp()
    {
        ICPInitTest::SetUp();
    }

    virtual void TearDown()
    {
        ICPInitTest::TearDown();
    }

    shared_ptr<base_Layer> to_; ///< pointer to test object
    Signal sig_;
};

#else // __WITH_PCL
    #warning "Skipping building ICP layer unittests"
#endif // __WITH_PCL

} // annonymous namespace for test fixtures

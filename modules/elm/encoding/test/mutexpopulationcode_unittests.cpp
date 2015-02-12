#include "elm/encoding/populationcode.h"

#include <opencv2/highgui.hpp>

#include "elm/core/cv/mat_utils.h"
#include "elm/core/layerconfig.h"
#include "elm/layers/layerfactory.h"
#include "elm/core/signal.h"
#include "elm/encoding/orientation.h"
#include "elm/io/synth.h"
#include "elm/ts/ts.h"

using std::string;
using cv::Mat1f;
using namespace elm;

namespace {

class MutexPopulationCodeTest : public testing::Test
{
protected:
    void virtual SetUp()
    {
        in_ = Mat1f::ones(3, 1);
        to_.State(in_);
    }

    MutexPopulationCode to_; ///< test object
    Mat1f in_;
};

TEST_F(MutexPopulationCodeTest, PopCode_dims) {

    EXPECT_MAT_DIMS_EQ(to_.PopCode(), cv::Size2i(in_.cols*2, in_.rows));

    in_ = Mat1f::ones(3, 2);
    to_.State(in_);
    EXPECT_MAT_DIMS_EQ(to_.PopCode(), cv::Size2i(in_.cols*2, in_.rows));
}

TEST_F(MutexPopulationCodeTest, PopCode_ones) {

    Mat1f pc = to_.PopCode();
    for(int i=0; i<pc.rows; i++) {

        EXPECT_FLOAT_EQ(pc(i), i%2==0? 0.f : 1.f);
    }
}

TEST_F(MutexPopulationCodeTest, PopCode_zeros) {

    in_ = Mat1f::zeros(in_.rows, in_.cols);
    to_.State(in_);
    Mat1f pc = to_.PopCode();

    for(int i=0; i<pc.rows; i++) {

        EXPECT_FLOAT_EQ(pc(i), i%2==0? 1.f : 0.f);
    }
}

const string NAME_STIMULUS  = "in";
const string NAME_POP_CODE   = "pc";

/**
 * @brief class for testing the initializations of the layer interface of MutexPopulationCode
 */
class MutexPopulationCodeLayerInitTest : public MutexPopulationCodeTest
{
protected:
    void virtual SetUp()
    {
        MutexPopulationCodeTest::SetUp();

        LayerConfig cfg;
        cfg.Input(MutexPopulationCode::KEY_INPUT_STIMULUS, NAME_STIMULUS);
        cfg.Output(MutexPopulationCode::KEY_OUTPUT_POP_CODE, NAME_POP_CODE);

        to_ptr_ = LayerFactory::CreateShared("MutexPopulationCode", cfg, cfg);
    }

    std::shared_ptr<base_Layer > to_ptr_; ///< test object
};

TEST_F(MutexPopulationCodeLayerInitTest, IONames)
{
    LayerConfig cfg;
    EXPECT_THROW(MutexPopulationCode().IONames(cfg), std::exception);

    cfg.Input(MutexPopulationCode::KEY_INPUT_STIMULUS, NAME_STIMULUS);
    EXPECT_THROW(MutexPopulationCode().IONames(cfg), std::exception) << "still missing required output";

    cfg.Output(MutexPopulationCode::KEY_OUTPUT_POP_CODE, NAME_POP_CODE);
    EXPECT_NO_THROW(MutexPopulationCode().IONames(cfg)) << "Are all required IO names present?";
}

/**
 * @brief class for testing the layer interface of MutexPopulationCode
 * The live tests
 */
class MutexPopulationCodeLayerTest : public MutexPopulationCodeLayerInitTest
{
protected:
    void virtual SetUp()
    {
        MutexPopulationCodeLayerInitTest::SetUp();
        sig_.Append(NAME_STIMULUS, in_);

        to_ptr_->Activate(sig_);
        to_ptr_->Response(sig_);
    }

    void virtual TearDown()
    {
        sig_.Clear();
    }

    Signal sig_;
};

TEST_F(MutexPopulationCodeLayerTest, IONames_Wrong) {

    EXPECT_MAT_DIMS_EQ(sig_.MostRecentMat1f(NAME_POP_CODE), cv::Size2i(in_.cols*2, in_.rows));

    in_ = Mat1f::ones(3, 2);
    sig_.Append(NAME_STIMULUS, in_);
    to_ptr_->Activate(sig_);
    to_ptr_->Response(sig_);
    to_.State(in_);

    EXPECT_MAT_DIMS_EQ(sig_.MostRecentMat1f(NAME_POP_CODE), cv::Size2i(in_.cols*2, in_.rows));
}

TEST_F(MutexPopulationCodeLayerTest, Response_dims) {

    EXPECT_MAT_DIMS_EQ(sig_.MostRecentMat1f(NAME_POP_CODE), cv::Size2i(in_.cols*2, in_.rows));

    in_ = Mat1f::ones(3, 2);
    sig_.Append(NAME_STIMULUS, in_);
    to_ptr_->Activate(sig_);
    to_ptr_->Response(sig_);
    to_.State(in_);

    EXPECT_MAT_DIMS_EQ(sig_.MostRecentMat1f(NAME_POP_CODE), cv::Size2i(in_.cols*2, in_.rows));
}

TEST_F(MutexPopulationCodeLayerTest, PopCode_ones) {

    Mat1f pc = sig_.MostRecentMat1f(NAME_POP_CODE);
    for(int i=0; i<pc.rows; i++) {

        EXPECT_FLOAT_EQ(pc(i), i%2==0? 0.f : 1.f);
    }
}

TEST_F(MutexPopulationCodeLayerTest, PopCode_zeros) {

    in_ = Mat1f::zeros(in_.rows, in_.cols);

    sig_.Append(NAME_STIMULUS, in_);
    to_ptr_->Activate(sig_);
    to_ptr_->Response(sig_);
    to_.State(in_);

    to_.State(in_);
    Mat1f pc = sig_.MostRecentMat1f(NAME_POP_CODE);

    for(int i=0; i<pc.rows; i++) {

        EXPECT_FLOAT_EQ(pc(i), i%2==0? 1.f : 0.f);
    }
}

} // anonymous namespace

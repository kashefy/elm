#include "layers/weightedsum.h"

#include "gtest/gtest.h"

#include <memory>

#include "core/exception.h"
#include "core/layerconfig.h"
#include "core/signal.h"
#include "ts/ts.h"

using namespace std;
using namespace cv;
using namespace sem;

class WeightedSumTest : public ::testing::Test
{
public:
    static const string NAME_STIMULUS;
    static const string NAME_RESPONSE;

protected:
    virtual void SetUp()
    {
        to_.reset(new WeightedSum());

        config_ = LayerConfig();

        // params
        PTree params;
        params.add(WeightedSum::PARAM_A, -1.f);
        params.add(WeightedSum::PARAM_B, 2.f);
        config_.Params(params);

        // IO
        config_.Input(WeightedSum::KEY_INPUT_STIMULUS, NAME_STIMULUS);
        config_.Output(WeightedSum::KEY_OUTPUT_RESPONSE, NAME_RESPONSE);

        to_.reset(new WeightedSum(config_));
    }

    unique_ptr<base_Layer> to_; ///< test object
    LayerConfig config_;        ///< default config for tests
};
const string WeightedSumTest::NAME_STIMULUS = "in";
const string WeightedSumTest::NAME_RESPONSE = "out";

TEST_F(WeightedSumTest, Reset_EmptyConfig)
{
    EXPECT_THROW(to_->Reset(LayerConfig()), boost::property_tree::ptree_bad_path);
}

TEST_F(WeightedSumTest, Activate)
{
    Signal signal;
    // feed input into signal object
    EXPECT_FALSE(signal.Exists(NAME_STIMULUS));
    signal.Append(NAME_STIMULUS, Mat1f::ones(3, 2));
    EXPECT_TRUE(signal.Exists(NAME_STIMULUS));

    // compute response
    EXPECT_FALSE(signal.Exists(NAME_RESPONSE));
    to_->Activate(signal);
    to_->Response(signal);
    EXPECT_TRUE(signal.Exists(NAME_RESPONSE)) << "Resonse missing";

    // Check response dimensions
    Mat1f response = signal[NAME_RESPONSE][0];
    EXPECT_MAT_DIMS_EQ(response, Size(1, 3));

    // Check response values
    float a = config_.Params().get<float>(WeightedSum::PARAM_A);
    float b = config_.Params().get<float>(WeightedSum::PARAM_B);
    for(int r=0; r<response.rows; r++) {

        EXPECT_EQ(a+b, response(r));
    }
}
#include "layers/weightedsum.h"

#include "gtest/gtest.h"

#include <memory>

#include "core/exception.h"
#include "core/layerconfig.h"
#include "core/signal.h"

using namespace std;
using namespace cv;

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

    }

    unique_ptr<base_Layer> to_; ///< test object
    LayerConfig config_;        ///< default config for tests
};
const string WeightedSumTest::NAME_STIMULUS = "stimulus";
const string WeightedSumTest::NAME_RESPONSE = "response";

TEST_F(WeightedSumTest, Reset)
{
    EXPECT_THROW(to_->Reset(LayerConfig()), ExceptionNotImpl);
}

TEST_F(WeightedSumTest, Configure)
{
    to_.reset(new WeightedSum(config_));

    Signal signal;
    EXPECT_FALSE(signal.Exists(NAME_STIMULUS));
    signal.Append(NAME_STIMULUS, Mat1f::ones(3, 2));
    EXPECT_TRUE(signal.Exists(NAME_STIMULUS));
    EXPECT_FALSE(signal.Exists(NAME_RESPONSE));

    to_->Stimulus(signal);
    to_->Apply();
    to_->Response(signal);
}

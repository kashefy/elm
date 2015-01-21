#include "sem/neuron/zneuron.h"

#include <vector>

#include "sem/ts/ts.h"
#include "sem/ts/fakeevidence.h"

using namespace cv;

namespace {


class ZNeuronTest : public testing::Test
{
protected:
    virtual void SetUp()
    {
        nb_features_ = 50;
        to_ = ZNeuron();
        to_.Init(nb_features_, 3);
    }

    ZNeuron to_;        ///< test object
    int nb_features_;
};

TEST_F(ZNeuronTest, InitialState)
{
    ZNeuron to;
    EXPECT_TRUE(to.Weights().empty());
    EXPECT_MAT_EQ(to.Bias(), Mat1f::zeros(1, 1));
}

TEST_F(ZNeuronTest, Init)
{
    // check weights
    Mat1f w = to_.Weights();
    EXPECT_FALSE(w.empty());
    EXPECT_MAT_DIMS_EQ(w, Size2i(nb_features_, 1));
    EXPECT_MAT_TYPE(w, CV_32F);

    EXPECT_MAT_LT(w, 0.f) << "Encountered weights > 0";

    // check weights are not all identical
    Mat m, s;
    cv::meanStdDev(w, m, s);
    EXPECT_NE(s.at<float>(0), 0);

    // check bias
    EXPECT_MAT_DIMS_EQ(to_.Bias(), Mat1f::zeros(1, 1));
}

TEST_F(ZNeuronTest, Predict)
{
    const int N=20;
    for(int i=0; i<N; i++) {

        // setup evidence vector
        Mat1f features(1, nb_features_);
        randn(features, 0, 1);
        Mat u = to_.Predict(features > 0);
        EXPECT_MAT_DIMS_EQ(u, Mat::zeros(1, 1, u.type())) << "Expecting scalar result from Predict method";
        EXPECT_MAT_TYPE(u, CV_32F) << "Unexpected Mat type";
        EXPECT_LT(u.at<float>(0), 0);
    }
}

/**
 * @brief Test that prediction is stateless/static.
 * Repeated input yields repeated prediciotn results. No learning.
 */
TEST_F(ZNeuronTest, PredictStateless)
{
    const int N=20;
    FakeEvidence fake_evidence(nb_features_);
    Mat initial_u;
    for(int i=0; i<N; i++) {

        Mat u = to_.Predict(fake_evidence.next(0) > 0);
        EXPECT_MAT_DIMS_EQ(u, Mat::zeros(1, 1, u.type())) << "Expecting scalar result from Predict method";
        EXPECT_MAT_TYPE(u, CV_32F) << "Unexpected Mat type";
        EXPECT_LT(u.at<float>(0), 0);

        Mat state = to_.State();
        EXPECT_MAT_EQ(state, u);

        if(i == 0) {

            u.copyTo(initial_u);
        }
        else {

            EXPECT_MAT_EQ(u, initial_u) << "Prediction result changing for repeated input.";
        }
    }
}

TEST_F(ZNeuronTest, WeightsCopied)
{
    Mat1f w = to_.Weights();
    const Mat1f w_clone = to_.Weights().clone();

    w += 1.f;

    EXPECT_MAT_EQ(w_clone, to_.Weights());
}

TEST_F(ZNeuronTest, Learn_NoFire)
{
    const Mat1f initial_weights = to_.Weights().clone();
    for(int i=0; i<50; i++) {

        float bias_prev = to_.Bias()(0);

        to_.Predict( Mat1i::ones(1, nb_features_) > 0 );
        to_.Learn( Mat1i::zeros(1, 1) );
        EXPECT_MAT_EQ(initial_weights, to_.Weights());

        EXPECT_GT(bias_prev, to_.Bias()(0)) << "Bias not decaying";
    }
}

TEST_F(ZNeuronTest, Learn_AlwaysFire)
{
    const Mat1f initial_weights = to_.Weights().clone();
    for(int i=0; i<50; i++) {

        float bias_prev = to_.Bias()(0);

        to_.Predict( Mat1i::ones(1, nb_features_) > 0 );
        to_.Learn( Mat1i::ones(1, 1) );
        EXPECT_FALSE( Equal(initial_weights, to_.Weights()) );
        EXPECT_LT(bias_prev, to_.Bias()(0)) << "Bias not increasing";
    }
}

TEST_F(ZNeuronTest, Learn)
{
    Mat1f weights_prev = to_.Weights().clone();
    float bias_prev = to_.Bias()(0);
    FakeEvidence f(nb_features_);

    for(int i=0; i<50; i++) {

        to_.Predict( f.next(0) > 0 );
        to_.Learn( Mat1i::ones(1, 1) );

        EXPECT_FALSE( Equal(weights_prev, to_.Weights()) );
        for(int j=0; j<weights_prev.cols; j+=2) {

            EXPECT_GT(weights_prev(j+1), to_.Weights()(j+1)) << "Weight for non-spiking input potentiating.";
            EXPECT_LT(weights_prev(j), to_.Weights()(j)) << "Weight for spiking input decaying.";
        }
        EXPECT_LT(bias_prev, to_.Bias()(0)) << "Bias not increasing";

        weights_prev = to_.Weights().clone();
        bias_prev = to_.Bias()(0);
    }
}

/**
 * @brief Test clearing of state/history
 * TODO: Write a better test for this. May need exposing learning rates.
 */
TEST_F(ZNeuronTest, Clear)
{
    EXPECT_NO_THROW(to_.Clear());
}

} // namespace

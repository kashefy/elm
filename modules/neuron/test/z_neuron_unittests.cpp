#include "neuron/zneuron.h"

#include "ts/ts.h"

using namespace cv;

namespace {


class ZNeuronTest : public testing::Test
{
protected:
    virtual void SetUp()
    {
        nb_features = 5;
        to_ = ZNeuron();
        to_.init(nb_features, 3);
    }

    ZNeuron to_;        ///< test object
    int nb_features;
};

TEST_F(ZNeuronTest, InitialState)
{
    ZNeuron to;
    EXPECT_TRUE(to.Weights().empty());
    EXPECT_MAT_EQ(to.Bias(), MatF::zeros(1, 1));
}

TEST_F(ZNeuronTest, Init)
{
    // check weights
    MatF w = to_.Weights();
    EXPECT_FALSE(w.empty());
    EXPECT_MAT_DIMS_EQ(w, MatF::zeros(1, nb_features));
    EXPECT_MAT_TYPE(w, CV_32F);

    for(int i=0; i<w.cols; i++) {
        EXPECT_LT(w(i), 0) << "Encountered weight > 0 at i=" << i;
    }

    // check weights are not all identical
    Mat m, s;
    cv::meanStdDev(w, m, s);
    EXPECT_NE(s.at<float>(0), 0);

    // check bias
    EXPECT_MAT_DIMS_EQ(to_.Bias(), MatF::zeros(1, 1));
}

TEST_F(ZNeuronTest, Predict)
{
    const int N=1;
    for(int i=0; i<N; i++) {

        // setup evidence vector
        MatF features(1, nb_features);
        randn(features, 0, 1);
        Mat u = to_.Predict(features > 0);
        EXPECT_MAT_DIMS_EQ(u, Mat::zeros(1, 1, u.type())) << "Expecting scalar result from Predict method";
        EXPECT_MAT_TYPE(u, CV_32F) << "Unexpected Mat type";
        EXPECT_LT(u.at<float>(0), 0);
    }
}

TEST_F(ZNeuronTest, WeightsCopied)
{
    MatF w = to_.Weights();
    MatF w_clone = to_.Weights();

    w += 1.f;

    EXPECT_MAT_EQ(w_clone, to_.Weights());
}

} // namespace

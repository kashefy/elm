#include "neuron/zneuron.h"

using namespace cv;

ZNeuron::ZNeuron()
    : base_Learner(),
      history_(1, 1)
{
}

void ZNeuron::init(int nb_features, int len_history)
{
    weights_ = MatF(1, nb_features);

    // initialize weights with random negative values from a half-normal distribution
    const float MEAN=0.f, STD_DEV=1.f, SCALE=-0.01f;
    MatF random_vals(1, nb_features+1);   // add 1 for bias term
    randn(random_vals, MEAN, STD_DEV);
    random_vals = abs(random_vals) * SCALE;

    weights_ = random_vals(0, nb_features);
    bias_ = random_vals(nb_features);

    history_ = SpikingHistory(nb_features, len_history);
}

void ZNeuron::Learn()
{
}

Mat ZNeuron::Predict(const Mat &evidence)
{
    float u = bias_;  // membrane potential u

    history_.Advance();
    history_.Update(evidence != 0);

    // u = bh.w' * [x];

    // Sum subset of weights with spiking evidence
    MatF sub_weights(1, evidence.cols);
    sub_weights.setTo(weights_, evidence > 0);
    u += cv::sum(sub_weights)(0);

    return Mat(1, 1, CV_32FC1).setTo(u);
}



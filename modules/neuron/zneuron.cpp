#include "neuron/zneuron.h"

using namespace cv;

ZNeuron::ZNeuron()
    : base_Learner(),
      history_(1, 1)
{
}

void ZNeuron::init(int nb_features, int len_history)
{
    weights_ = MatF(1, nb_features+1); // add one for bias term at index 0

    // initialize weights randomly
    randn(weights_, 0.f, 1.f);
    weights_ = abs(weights_) * 0.01f;

    history_ = SpikingHistory(nb_features, len_history);
}

void ZNeuron::Learn()
{
}

Mat ZNeuron::Predict(const Mat &evidence)
{
    float u = 0.f;  // membrane potential u

    history_.Advance();
    history_.Update(evidence != 0);

    // u = bh.w' * [x];
    u += weights_(0);

    MatF weights_to_sum(1, evidence.cols);
    weights_to_sum.setTo(weights_.colRange(1, weights_.cols), evidence != 0);
    u += cv::sum(weights_to_sum)(0);

    return Mat(1, 1, CV_32FC1).setTo(u);
}



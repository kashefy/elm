#include "neuron/zneuron.h"

using namespace cv;

ZNeuron::ZNeuron()
    : base_Learner()
{
}

void ZNeuron::init(int nb_features, int len_history)
{
    weights_ = MatF(1, nb_features+1); // add one for bias term at index 0

    // initialize weights randomly
    randn(weights_, 0.f, 1.f);
    weights_ = abs(weights_) * 0.01f;
}

void ZNeuron::Learn()
{
}

Mat ZNeuron::Predict(const Mat &evidence)
{
    return Mat::zeros(1, 1, CV_32FC1);
}



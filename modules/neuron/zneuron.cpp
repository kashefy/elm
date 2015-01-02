#include "neuron/zneuron.h"

using namespace cv;

ZNeuron::ZNeuron()
    : base_Learner(),
      bias_(0.f),
      history_(1, 1)
{
}

void ZNeuron::Init(int nb_features, int len_history)
{
    weights_ = Mat1f(1, nb_features);

    const float MEAN=0.f, STD_DEV=1.f, SCALE=-0.01f;
    Mat1f random_vals(1, nb_features+1);   // add 1 for bias term
    randn(random_vals, MEAN, STD_DEV);
    random_vals = abs(random_vals) * SCALE;

    weights_ = random_vals.colRange(0, nb_features);
    bias_ = random_vals(nb_features);

    history_ = SpikingHistory(nb_features, len_history);
}

void ZNeuron::Learn(const Mat &target)
{
    const double WEIGHT_LIMIT = 5.0;
    double limit_factor;
    double delta_w, w_old, w0, w;
    double eta;  // learning rate
    const bool has_fired = countNonZero(target) > 0;

    // update w0/bias
    // determine limit factor
    w_old = bias_;
    eta = 0.01f; // TODO make parameter and use adaptive learning rate
    limit_factor = exp( -max(w_old, log(eta)) );

    // bh.w(1,i) = old_w0 + nEta * (C(i) * (1 - exp(old_w0)) - (1-C(i)) * exp(old_w0)) * ...
        // limit_factor;
    // Nessler's (2010) STDP equation (12)
    delta_w = eta * limit_factor;
    delta_w *= has_fired? 1-exp(w_old) : -exp(w_old);
    w0 = w_old + delta_w;

    // bh.w(1,i) = max(bh.w(1,i), -bh.limit);
    bias_ = static_cast<float>(max(w0, -WEIGHT_LIMIT));
    //m_biasLearningRate.update(m_nBias);         // TODO adaptive learning rate

    // update all other weights
    if(has_fired) {

        Mat1f weights_old = weights_;
        Mat1f _eta(weights_.size(), 0.01f); // TODO: adaptive learing rate per weight
        Mat1f _eta_log;
        log(_eta, _eta_log);
        Mat1f _limit_factor;
        exp(-cv::max(weights_old, _eta_log), _limit_factor);

        Mat1f _delta_w = _limit_factor.mul(_eta);

        Mat1f _exp_w_old;
        exp(weights_old, _exp_w_old);

        // compute weight deltas according to afferent spiking
        Mat1f _delta_w_cond = _delta_w.mul(_exp_w_old, -1.);
        add(_delta_w_cond, _delta_w, _delta_w_cond, history_.Recent()); // mask operation by recenlty spiking afferents

        // bh.w(2:(bh.dim+1),i) = old_w + delta .* C(i) .* limit_factor;
        // bh.w(2:(bh.dim+1),i) = max(bh.w(2:(bh.dim+1),i), -bh.limit);

        weights_ = weights_old + _delta_w_cond;
        weights_.setTo(-WEIGHT_LIMIT, weights_ < -WEIGHT_LIMIT);

        //m_arrLearningRate[wi].update(w);         // TODO: adaptive learning rate
    }// if not fired, no change
}

Mat ZNeuron::Predict(const Mat &evidence)
{
    u_ = bias_;  // membrane potential u

    history_.Advance();
    history_.Update(evidence != 0);

    // u = bh.w' * [x];

    // Sum subset of weights with spiking evidence
    Mat1f sub_weights = Mat1f::zeros(1, static_cast<int>(evidence.total()));
    add(sub_weights, weights_, sub_weights, evidence > 0);

    u_ += sum(sub_weights)(0);

    return State();
}

Mat ZNeuron::State() const
{
    return Mat(1, 1, CV_32FC1, u_);
}

Mat1f ZNeuron::Weights() const
{
    return weights_;
}

Mat1f ZNeuron::Bias() const
{
    return Mat1f(1, 1, bias_);
}

void ZNeuron::Clear()
{
    history_.Reset();
}


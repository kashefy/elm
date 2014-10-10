#include "neuron/zneuron.h"

using namespace cv;

ZNeuron::ZNeuron()
    : base_Learner(),
      bias_(0.f),
      history_(1, 1)
{
}

void ZNeuron::init(int nb_features, int len_history)
{
    weights_ = MatF(1, nb_features);

    const float MEAN=0.f, STD_DEV=1.f, SCALE=-0.01f;
    MatF random_vals(1, nb_features+1);   // add 1 for bias term
    randn(random_vals, MEAN, STD_DEV);
    random_vals = abs(random_vals) * SCALE;

    weights_ = random_vals.colRange(0, nb_features);
    bias_ = random_vals(nb_features);

    history_ = SpikingHistory(nb_features, len_history);
}

void ZNeuron::Learn()
{
    double limit_factor;
    double delta_w, w_old, w0, w;
    double eta;  // learning rate

    // determine limit factor
    w_old = bias_;
    eta = 0.01f; // TODO make parameter
    limit_factor = exp( -max(w_old, log(eta)) );

    // update w0
    // bh.w(1,i) = old_w0 + nEta * (C(i) * (1 - exp(old_w0)) - (1-C(i)) * exp(old_w0)) * ...
        // limit_factor;
    // Nessler's (2010) STDP equation (12)
    delta_w = eta * limit_factor;
    delta_w *= has_fired_? 1-exp(w_old) : -exp(w_old);
    w0 = w_old + delta_w;

    // bh.w(1,i) = max(bh.w(1,i), -bh.limit);
    bias_ = static_cast<float>(max(w0, -5.0)); // TODO: define const weight limit
    //m_biasLearningRate.update(m_nBias);         // TODO adaptive learning rate

    // update all other weights
    if(has_fired_) {

        // TODO: vectorize
        for(int wi=0; wi<weights_.cols; ++wi) {

            w_old = weights_(wi);
            //nEta = m_arrLearningRate[wi].getEta(); // TODO: adaptive learing rate per weight
            limit_factor = exp(-max(w_old, log(eta)));

            delta_w = eta * limit_factor;
            delta_w *= history_.Recent(wi)? 1-exp(w_old) : -exp(w_old);

            // bh.w(2:(bh.dim+1),i) = old_w + delta .* C(i) .* limit_factor;
            // bh.w(2:(bh.dim+1),i) = max(bh.w(2:(bh.dim+1),i), -bh.limit);

            w = w_old + delta_w;
            w = max(w, -5.0); // TODO: define const weight limit
            weights_(wi) = static_cast<float>(w);
            //m_arrLearningRate[wi].update(w);         // TODO: adaptive learning rate
        }
    }// if not fired, no change
}

Mat ZNeuron::Predict(const Mat &evidence)
{
    float u = bias_;  // membrane potential u

    history_.Advance();
    history_.Update(evidence != 0);

    // u = bh.w' * [x];

    // Sum subset of weights with spiking evidence
    MatF sub_weights = MatF::zeros(1, evidence.cols);
    add(sub_weights, weights_, sub_weights, evidence > 0);

    //sub_weights.setTo(weights_, evidence > 0);
    u += sum(sub_weights)(0);

    return Mat(1, 1, CV_32FC1).setTo(u);
}

MatF ZNeuron::Weights() const
{
    return weights_;
}

MatF ZNeuron::Bias() const
{
    return MatF(1, 1).setTo(bias_);
}

void ZNeuron::LetFire(bool let_fire)
{
    has_fired_ = let_fire;
}

bool ZNeuron::HasFired() const
{
    return has_fired_;
}


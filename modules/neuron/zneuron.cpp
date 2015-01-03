#include "neuron/zneuron.h"

#include "core/exception.h"

using namespace cv;

ZNeuron::ZNeuron()
    : base_Learner(),
      weights_all_(1, 1, 0.f),
      bias_(weights_all_.clone()),
      history_all_(1, 1),
      history_afferents_(1, 1),
      history_self_(1, 1)
{
}

void ZNeuron::Init(int nb_features, int len_history)
{
    weights_all_ = Mat1f(1, nb_features+1); // add 1 for bias term

    const float MEAN=0.f, STD_DEV=1.f, SCALE=-0.01f;
    randn(weights_all_, MEAN, STD_DEV); // todo plug in the right stddev instead of multiplying
    weights_all_ = abs(weights_all_) * SCALE;

    weights_ = weights_all_.colRange(1, nb_features+1); // used for easier referencing of weights excluding bias term
    bias_    = weights_all_.col(0);

    history_all_ = SpikingHistory(nb_features+1, len_history);  // add 1 for self spiking
    history_afferents_ = history_all_.ColRange(1, nb_features+1);
    history_self_ = history_all_.ColRange(0, 1);
}

void ZNeuron::Learn(const Mat &target)
{
    if(countNonZero(target) > 0) { // this neuron has fired recently

        history_self_.Update(Mat1b::ones(1, 1));
        Update(weights_all_, history_all_.Recent());
    }
    else {

        history_self_.Reset();
        Update(bias_, history_self_.Recent());
    }
}

void ZNeuron::Update(Mat &weights, const Mat &has_spiked_recently) const
{
    const double WEIGHT_LIMIT = 5.0;

    Mat1f weights_old = weights;      // this does not perform a deep copy
    Mat1f _eta(weights.size(), 0.01f); // TODO: adaptive learing rate per weight
    Mat1f _eta_log;
    log(_eta, _eta_log);
    Mat1f _limit_factor;
    exp(-max(weights_old, _eta_log), _limit_factor);

    Mat1f _delta_w = _limit_factor.mul(_eta);

    Mat1f _exp_w_old;
    exp(weights_old, _exp_w_old);

    // compute weight deltas according to afferent spiking
    Mat1f _delta_w_cond = _delta_w.mul(_exp_w_old, -1.);
    add(_delta_w_cond, _delta_w, _delta_w_cond, has_spiked_recently); // mask operation by recenlty spiking afferents

    // bh.w(2:(bh.dim+1),i) = old_w + delta .* C(i) .* limit_factor;
    // bh.w(2:(bh.dim+1),i) = max(bh.w(2:(bh.dim+1),i), -bh.limit);

    weights = weights_old + _delta_w_cond;
    weights.setTo(-WEIGHT_LIMIT, weights < -WEIGHT_LIMIT);

    //m_arrLearningRate[wi].update(w);         // TODO: adaptive learning rate
}


Mat ZNeuron::Predict(const Mat &evidence)
{
    u_ = weights_all_(0);  // membrane potential u

    history_afferents_.Advance();
    history_afferents_.Update(evidence != 0);

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
    return weights_.clone();
}

Mat1f ZNeuron::Bias() const
{
    return bias_.clone();
}

void ZNeuron::Clear()
{
    history_all_.Reset();
}


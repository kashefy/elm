#include "neuron/wtapoisson.h"

#include "core/distributionsampler.h"

using namespace std;
using namespace cv;

WTAPoisson::WTAPoisson(float max_frequency, float delta_t_msec)
    : base_WTA(delta_t_msec),
      lambda_(max_frequency)
{
    NextSpikeTime();

}

void WTAPoisson::NextSpikeTime()
{
    next_spike_time_sec_ = sem::randexp(lambda_);
}

Mat WTAPoisson::Compete(vector<shared_ptr<base_Learner> > &learners)
{
    // results represent which learner fired (1) and which were inhibited (0)
    Mat1i winners = Mat1i::zeros(1, static_cast<int>(learners.size()));

    // time to spike or still in refractory period
    if(next_spike_time_sec_ < delta_t_sec_) { // time to spike

        // Distribution of learner states
        Mat1f soft_max = LearnerStateDistr(learners);

        DistributionSampler1D sampler;
        sampler.pdf(soft_max);

        winners(sampler.Sample()) = 1;

        NextSpikeTime();
    }
    else { // refractory period

        next_spike_time_sec_ -= delta_t_sec_;
    }

    return winners > 0;
}

cv::Mat WTAPoisson::LearnerStateDistr(const std::vector<std::shared_ptr<base_Learner> > &learners) const
{
    int nb_learners = static_cast<int>(learners.size());
    Mat1f u(1, nb_learners);
    for(int i=0; i<nb_learners; i++) {

        u(i) = learners[i]->State().at<float>(0);
    }

    // normalize learner state distribution
    Mat mean, s;
    meanStdDev(u, mean, s);

    u -= static_cast<float>(mean.at<double>(0));
    Mat1f soft_max;
    exp(u, soft_max);
    soft_max /= sum(soft_max)(0);

    return soft_max;
}



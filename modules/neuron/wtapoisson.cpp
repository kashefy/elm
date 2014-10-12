#include "neuron/wtapoisson.h"

#include "core/distributionsampler.h"

using namespace cv;

WTAPoisson::WTAPoisson(float max_frequency, float delta_t_msec)
    : base_WTA(delta_t_msec),
      lambda_(1./max_frequency)
{
    next_spike_time_ = NextSpikeTime();
}

float WTAPoisson::NextSpikeTime() const
{
    return sem::randexp(lambda_);
}

Mat WTAPoisson::Compete(std::vector<base_Learner> &learners)
{
    int nb_learners = static_cast<int>(learners.size());
    // results represent which learner fired (1) and which were inhibited (0)
    MatI winners = MatI::zeros(1, nb_learners);

    // time to spike or still in refractory period
    if(next_spike_time_ < delta_t_msec_) { // time to spike

        // Distribution of learner states
        MatF u(1, nb_learners);
        for(int i=0; i<nb_learners; i++) {

            u.row(i) = learners[i].State();
        }

        // normalize learner state distribution
        Mat mean, s;
        meanStdDev(u, mean, s);

        u -= static_cast<float>(mean.at<double>(0));
        MatF soft_max;
        exp(u, soft_max);
        soft_max /= sum(soft_max)(0);

        DistributionSampler1D sampler;
        sampler.pdf(soft_max);

        winners(sampler.Sample()) = 1;
    }
    else { // refractory period

        next_spike_time_ -= delta_t_msec_;
    }

    return winners > 0;
}


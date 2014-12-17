#include "neuron/wtapoisson.h"

#include "core/exception.h"
#include "neuron/zneuron.h"
#include "ts/ts.h"

using namespace std;
using namespace cv;
using namespace sem;

class WTAPoissonTest : public testing::Test
{
protected:
    WTAPoissonTest()
          : to_(0, 0)
    {
    }

    virtual void SetUp()
    {
        max_frequency_ = 1e5; // very high frequency
        delta_t_msec_ = 1.f;

        to_ = WTAPoisson(max_frequency_, delta_t_msec_);

        const int nb_learners = 3;
        for(int i=0; i<nb_learners; i++) {

            shared_ptr<ZNeuron> p(new ZNeuron);
            p->init(1, 1);
            p->Predict(Mat1i::ones(1, 1) > 0);
            learners_.push_back(p);
        }
    }

    WTAPoisson to_;         ///< test object
    float max_frequency_;   ///< max. WTA firing rate
    float delta_t_msec_;     ///< time resolution in milliseonds
    vector<shared_ptr<base_Learner> > learners_; ///< vector of spiking learners
};

/**
 * @brief With WTA's max firing rate set very high we expect 1 learner to always fire
 */
TEST_F(WTAPoissonTest, AlwaysFire)
{
    const int N=100;
    for(int i=0; i<N; i++) {

        Mat outcome = to_.Compete(learners_);
        EXPECT_EQ(countNonZero(outcome), 1) << "Expecting 1 learner to fire.";
    }
}

/**
 * @brief Test when WTA is configured to never fire
 * (e.g. with WTA's zero firing rate set to zero)
 */
TEST_F(WTAPoissonTest, NeverFire)
{
    const int N=100;

    vector<WTAPoisson> wtas;
    wtas.push_back(WTAPoisson(0.f, 1.f)); // zero freq.
    wtas.push_back(WTAPoisson(1e5, 0.f)); // zero deltaT.
    int nb_wta = static_cast<int>(wtas.size());

    WTAPoisson to(0.f, 1.f);
    for(int i=0; i<N; i++) {

        for(int j=0; j<nb_wta; j++) {

            Mat outcome = wtas[j].Compete(learners_);
            EXPECT_EQ(countNonZero(outcome), 0) << "Learner fired.";
        }
    }
}

TEST_F(WTAPoissonTest, FiringRate)
{
    const int N=1e4;
    for(float f=20.f; f<=80.f; f+=20.f) {

        WTAPoisson to(f, delta_t_msec_);

        int spike_count = 0;
        for(int i=0; i<N; i++) {

            Mat outcome = to.Compete(learners_);
            spike_count += countNonZero(outcome);
        }

        float rate = spike_count/static_cast<float>(N);
        rate *= 1000; // since time resolution was in milliseconds
        EXPECT_NEAR(rate, f, 10.f); //TODO: tighten this up a bit
    }
}

TEST_F(WTAPoissonTest, LearnerStateDistr)
{
    Mat distr = to_.LearnerStateDistr(learners_);
    EXPECT_MAT_TYPE(distr, CV_32F);
    int nb_learners = static_cast<int>(learners_.size());
    EXPECT_MAT_DIMS_EQ(distr, Mat1f(1, nb_learners));

    Mat1f u(1, nb_learners);
    for(int i=0; i<nb_learners; i++) {

        u(i) = learners_[i]->State().at<float>(0);

        EXPECT_GE(distr.at<float>(i), 0);
    }

    double min_val, max_val;
    int min_idx1[2] = {-1, -1}, max_idx1[2] = {-1, -1};

    minMaxIdx(distr, &min_val, &max_val, min_idx1, max_idx1);
    EXPECT_EQ(min_idx1[0], max_idx1[0]);
    EXPECT_NE(min_idx1[1], max_idx1[1]);

    int min_idx2[2] = {-1, -1}, max_idx2[2] = {-1, -1};
    minMaxIdx(u, &min_val, &max_val, min_idx2, max_idx2);
    EXPECT_EQ(min_idx2[0], max_idx2[0]);
    EXPECT_NE(min_idx2[1], max_idx2[1]);

    EXPECT_MAT_EQ(Mat1i(1, 2, min_idx1), Mat1i(1, 2, min_idx2));
    EXPECT_MAT_EQ(Mat1i(1, 2, max_idx1), Mat1i(1, 2, max_idx2));
}

TEST_F(WTAPoissonTest, NoLearners)
{
    EXPECT_THROW(to_.LearnerStateDistr(vector<shared_ptr<base_Learner> >()), ExceptionBadDims);
}




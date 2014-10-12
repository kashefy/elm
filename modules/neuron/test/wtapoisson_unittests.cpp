#include "neuron/wtapoisson.h"

#include "neuron/zneuron.h"
#include "ts/ts.h"

using namespace std;

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
        delta_t_msec = 1.f;

        to_ = WTAPoisson(max_frequency_, delta_t_msec);

        const int nb_learners = 3;
        for(int i=0; i<nb_learners; i++) {

            shared_ptr<ZNeuron> p(new ZNeuron);
            p->init(1, 1);
            learners_.push_back(p);
        }
    }

    WTAPoisson to_;         ///< test object
    float max_frequency_;   ///< max. WTA firing rate
    float delta_t_msec;     ///< time resolution in milliseonds
    vector<shared_ptr<base_Learner> > learners_; ///< vector of spiking learners
};

/**
 * @brief With WTA's max firing rate set very high we expect 1 learner to always fire
 */
TEST_F(WTAPoissonTest, AlwaysFire)
{
    const int N=100;
    for(int i=0; i<N; i++) {

        cv::Mat outcome = to_.Compete(learners_);
        EXPECT_EQ(cv::countNonZero(outcome), 1) << "Expecting 1 learner to fire.";
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

            cv::Mat outcome = wtas[j].Compete(learners_);
            EXPECT_EQ(cv::countNonZero(outcome), 0) << "Learner fired.";
        }
    }
}


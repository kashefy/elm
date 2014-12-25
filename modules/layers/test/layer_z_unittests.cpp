/** @file testing layer z class
 * Test coverage will be distributed into the following:
 * test layer initialization using fixutres
 * test layer parameter validation using value-parameterized tests
 * test layer application with fake input data using fixtures
 *
 * in order to define a common base test class and utilize different
 * gtest test fixutres as well as value-parameterization we'll define
 * the a bast class for testing in the form of a mixin.
 */
#include "layers/layer_z.h"

#include "core/exception.h"
#include "core/ptree_utils.h"
#include "core/signal.h"
#include "ts/ts.h"
#include "ts/fakeevidence.h"

using namespace std;
using namespace cv;
using namespace sem;

// name of keys in signal
const string NAME_INPUT_SPIKES  = "in";          ///< no. of afferent spikes
const string NAME_OUTPUT_SPIKES = "out";         ///< no. of output spikes/size of output layer
const string NAME_OUTPUT_MEM_POT = "mem_pot";    ///< membrane potential

/**
 * @brief mixin for testing layer Z, the main, SEM learning algorithm
 * Because we expect to test using fixutres as well as parametrized tests
 * Since these two test types are derived differently mixins solves the problem of defining a common base
 * The solution is adopted from this StackOverflow post:
 * @see http://stackoverflow.com/questions/3152326/google-test-parameterized-tests-which-use-an-existing-test-fixture-class
 *
 * T specifies test type (fixture or parameterized)
 */
template <class T> class LayerZTestBase : public T
{
protected:
    virtual void SetUp()
    {
        nb_afferents_ = 50;
        params_ = PTree();
        params_.put(LayerZ::PARAM_NB_AFFERENTS, nb_afferents_);
        params_.put(LayerZ::PARAM_NB_OUTPUT_NODES, 10);
        config_.Params(params_);

        // IO
        config_.Input(LayerZ::KEY_INPUT_SPIKES, NAME_INPUT_SPIKES);

        config_.Output(LayerZ::KEY_OUTPUT_SPIKES, NAME_OUTPUT_SPIKES);
        config_.Output(LayerZ::KEY_OUTPUT_MEMBRANE_POT, NAME_OUTPUT_MEM_POT);
    }

    // members
    LayerConfig config_;    ///< default layer configuration
    PTree params_;          ///< default params

    int nb_afferents_;
};

/**
 * @brief class for testing layer z initialization routines
 */
class LayerZInitTest : public LayerZTestBase<testing::Test >
{
protected:
};

/**
 * @brief test validation of missing params
 */
TEST_F(LayerZInitTest, MissingParams)
{
    // remove key to output nodes
    params_.erase(LayerZ::PARAM_NB_OUTPUT_NODES);
    config_.Params(params_);

    EXPECT_THROW(LayerZ().Reset(config_), std::exception);
}

TEST_F(LayerZInitTest, IONames)
{
    LayerConfig cfg;
    cfg.Params(config_.Params());
    EXPECT_THROW(LayerZ().IONames(cfg), std::exception);

    cfg.Input(LayerZ::KEY_INPUT_SPIKES, NAME_INPUT_SPIKES);
    EXPECT_THROW(LayerZ().IONames(cfg), std::exception) << "still missing required output spikes";

    cfg.Output(LayerZ::KEY_OUTPUT_SPIKES, NAME_OUTPUT_SPIKES);
    EXPECT_NO_THROW(LayerZ().IONames(cfg)) << "all required IO names present";
}

typedef std::pair<std::string, float > TParamPairSF; /// convinience typdef to use as test params below

/**
 * @brief Parameterized test for testing layer z parameter value validation
 * Using paramterized tests allows us to try different value more easily
 */
class LayerZParamsTest : public LayerZTestBase<testing::TestWithParam<TParamPairSF > >
{
protected:
};

/**
 * @brief use macro for instantiating value-parameterized test
 * if you're using QTCreator, don't mind the ..._EvalGenerator_ warning at the bottom
 */
INSTANTIATE_TEST_CASE_P(TestWithParams,
                        LayerZParamsTest,
                        testing::Values(TParamPairSF(LayerZ::PARAM_DELTA_T, -1.f),
                                        TParamPairSF(LayerZ::PARAM_DELTA_T, -0.f),
                                        TParamPairSF(LayerZ::PARAM_DELTA_T, 0.f),
                                        TParamPairSF(LayerZ::PARAM_LEN_HISTORY, -3),
                                        TParamPairSF(LayerZ::PARAM_LEN_HISTORY, 0),
                                        TParamPairSF(LayerZ::PARAM_NB_AFFERENTS, 0),
                                        TParamPairSF(LayerZ::PARAM_NB_AFFERENTS, -3),
                                        TParamPairSF(LayerZ::PARAM_WTA_FREQ, -0.001f),
                                        TParamPairSF(LayerZ::PARAM_WTA_FREQ, -1.f)));

TEST_P(LayerZParamsTest, InvalidParams)
{
    TParamPairSF tp = GetParam();
    params_.put(tp.first, tp.second);
    config_.Params(params_);
    EXPECT_THROW(LayerZ().Reset(config_), ExceptionValueError);
}

/**
 * @brief class for covering layer z methods assuming proper initialization, the live methods
 */
class LayerZTest : public LayerZTestBase<testing::Test >
{
protected:
    virtual void SetUp()
    {
        LayerZTestBase::SetUp();
        to_.Reset(config_);
        to_.IONames(config_);

        FakeEvidence stimuli(nb_afferents_);
        signal_.Append(NAME_INPUT_SPIKES, stimuli.next(0));
    }

    // members
    LayerZ to_;     ///< test object
    Signal signal_;
}

TEST_F(LayerZTest, ResponseDims)
{
    const int N=10;
    const nb_output_nodes = config_.Params().get(LayerZ::PARAM_NB_OUTPUT_NODES);
    for(int i=0; i<N; i++) {

        to_.Stimulus(signal_);
        to_.Apply();
        to_.Response(signal_);

        // check membrane potential
        Mat u = signal_.MostRecent(NAME_OUTPUT_MEM_POT);
        EXPECT_MAT_DIMS_EQ(u, Size(nb_output_nodes, 1)) << "Expecting scalar result from Predict method";
        EXPECT_MAT_TYPE(u, CV_32F) << "Unexpected Mat type";
        EXPECT_EQ(1, u.channels()) << "Expecting single-channel matrix";
        EXPECT_LT(u.at<float>(0), 0);

        // check output spikes
        Mat spikes = signal_.MostRecent(NAME_OUTPUT_SPIKES);
        EXPECT_MAT_DIMS_EQ(spikes, Size(nb_output_nodes, 1)) << "Expecting scalar result from Predict method";
        EXPECT_MAT_TYPE(spikes, CV_32F) << "Unexpected Mat type";
        EXPECT_EQ(1, spikes.channels()) << "Expecting single-channel matrix";
    }
}


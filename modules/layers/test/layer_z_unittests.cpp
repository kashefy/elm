#include "layers/layer_z.h"

#include "core/exception.h"
#include "core/ptree_utils.h"
#include "ts/ts.h"

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
    // members
    LayerZ to_;  ///< test object
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
    {
        LayerZ to;
        EXPECT_THROW(to.IONames(cfg), std::exception);
    }
    {
        cfg.Input(LayerZ::KEY_INPUT_SPIKES, NAME_INPUT_SPIKES);
        LayerZ to;
        EXPECT_THROW(to.IONames(cfg), std::exception) << "still missing required output spikes";
    }
    {
        cfg.Output(LayerZ::KEY_OUTPUT_SPIKES, NAME_OUTPUT_SPIKES);
        LayerZ to;
        EXPECT_NO_THROW(to.IONames(cfg)) << "all required IO names present";
    }
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

// if you're using QTCreator, don't mind the ..._EvalGenerator_ warning at the bottom
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

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
#include "core/cv/mat_utils_inl.h"
#include "core/boost/ptree_utils.h"
#include "core/signal.h"
#include "ts/ts.h"
#include "ts/fakeevidence.h"

using namespace std;
using namespace cv;
using namespace sem;

// name of keys in signal
const string NAME_INPUT_SPIKES   = "in";        ///< no. of afferent spikes
const string NAME_OUTPUT_SPIKES  = "out";       ///< no. of output spikes/size of output layer
const string NAME_OUTPUT_MEM_POT = "mem_pot";   ///< membrane potential
const string NAME_OUTPUT_WEIGHTS = "weights";   ///< neuron weights
const string NAME_OUTPUT_BIAS    = "bias";         ///< neuron weights

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
    LayerIONames io_names;
    EXPECT_THROW(LayerZ().IONames(io_names), std::exception);

    io_names.Input(LayerZ::KEY_INPUT_SPIKES, NAME_INPUT_SPIKES);
    EXPECT_THROW(LayerZ().IONames(io_names), std::exception) << "still missing required output spikes";

    io_names.Output(LayerZ::KEY_OUTPUT_SPIKES, NAME_OUTPUT_SPIKES);
    EXPECT_NO_THROW(LayerZ().IONames(io_names)) << "Are all required IO names present?";
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
};

TEST_F(LayerZTest, ResponseDims)
{
    const int N=10;
    const int nb_output_nodes = config_.Params().get<int>(LayerZ::PARAM_NB_OUTPUT_NODES);
    for(int i=0; i<N; i++) {

        to_.Activate(signal_);
        to_.Response(signal_);

        // check membrane potential
        Mat u = signal_.MostRecentMat(NAME_OUTPUT_MEM_POT);
        EXPECT_MAT_DIMS_EQ(u, Size(nb_output_nodes, 1)) << "Expecting scalar result from Predict method";
        EXPECT_MAT_TYPE(u, CV_32F) << "Unexpected Mat type";
        EXPECT_EQ(1, u.channels()) << "Expecting single-channel matrix";
        EXPECT_LT(u.at<float>(0), 0);

        // check output spikes
        Mat spikes = signal_.MostRecentMat(NAME_OUTPUT_SPIKES);
        EXPECT_MAT_DIMS_EQ(spikes, Size(nb_output_nodes, 1)) << "Expecting scalar result from Predict method";
        EXPECT_MAT_TYPE(spikes, CV_32F) << "Unexpected Mat type";
        EXPECT_EQ(1, spikes.channels()) << "Expecting single-channel matrix";
    }
}

TEST_F(LayerZTest, Clear)
{
    EXPECT_NO_THROW(to_.Clear());
}

/**
 * @brief Test layer's output is stateless and static when it's not learning anything.
 */
TEST_F(LayerZTest, StatelessMemPot)
{
    const int N=50;

    Mat u_initial;
    for(int i=0; i<N; i++) {

        to_.Activate(signal_);
        to_.Response(signal_);

        Mat u = signal_.MostRecentMat(NAME_OUTPUT_MEM_POT);

        if(i == 0) {

            u.copyTo(u_initial);
        }
        else {

            EXPECT_MAT_EQ(u, u_initial) << "\'u\' response is not static.";
        }
    }
}

/**
 * @brief test adding request to optional output of membrane potentials
 */
TEST_F(LayerZTest, OptionalOutput)
{
    LayerConfig cfg;
    cfg.Params(config_.Params());
    cfg.Input(LayerZ::KEY_INPUT_SPIKES, NAME_INPUT_SPIKES);
    cfg.Output(LayerZ::KEY_OUTPUT_SPIKES, NAME_OUTPUT_SPIKES);

    LayerZ to;
    to.Reset(cfg);
    EXPECT_NO_THROW(to.IONames(cfg)) << "Are all required IO names present?";

    to.Activate(signal_);
    {
        Signal signal_new;
        to.Response(signal_new);
        EXPECT_NO_THROW(to.IONames(cfg)) << "Are all required IO names present?";
        EXPECT_FALSE(signal_new.Exists(NAME_OUTPUT_MEM_POT));
        EXPECT_FALSE(signal_new.Exists(NAME_OUTPUT_WEIGHTS));
    }

    cfg.Output(LayerZ::KEY_OUTPUT_MEMBRANE_POT, NAME_OUTPUT_MEM_POT);
    EXPECT_NO_THROW(to.IONames(cfg)) << "Are all required IO names present?";
    to.Activate(signal_);
    {
        Signal signal_new;
        to.Response(signal_new);
        EXPECT_TRUE(signal_new.Exists(NAME_OUTPUT_MEM_POT));
        EXPECT_FALSE(signal_new.Exists(NAME_OUTPUT_WEIGHTS));
    }

    cfg.Output(LayerZ::KEY_OUTPUT_WEIGHTS, NAME_OUTPUT_WEIGHTS);
    EXPECT_NO_THROW(to.IONames(cfg)) << "Are all required IO names present?";
    to.Activate(signal_);
    {
        Signal signal_new;
        to.Response(signal_new);
        EXPECT_TRUE(signal_new.Exists(NAME_OUTPUT_MEM_POT));
        EXPECT_TRUE(signal_new.Exists(NAME_OUTPUT_WEIGHTS));
    }
}

/**
 * @brief Test layer's stimulus validation
 */
TEST_F(LayerZTest, Stimulus)
{
    signal_.Append(NAME_INPUT_SPIKES, Mat1f());
    EXPECT_THROW(to_.Activate(signal_), ExceptionBadDims);

    for(int r=0; r<=nb_afferents_*2; r++) {

        for(int c=0; c<=nb_afferents_*2; c++) {

            signal_.Append(NAME_INPUT_SPIKES, Mat1f::zeros(r, c));

            if(r*c==nb_afferents_) {

                EXPECT_NO_THROW(to_.Activate(signal_));
            }
            else {

                EXPECT_THROW(to_.Activate(signal_), ExceptionBadDims);
            }
        }
    }
}

TEST_F(LayerZTest, Activate)
{
    Mat1f spikes(1, nb_afferents_);
    randn(spikes, 0.f, 1.f);
    signal_.Append(NAME_INPUT_SPIKES, spikes);
    to_.Activate(signal_);
    EXPECT_FALSE(signal_.Exists(NAME_OUTPUT_SPIKES));
    to_.Response(signal_);
    EXPECT_TRUE(signal_.Exists(NAME_OUTPUT_SPIKES));
}

/**
 * @brief class for covering layer z neuron learning, weights and bias
 */
class LayerZLearnTest : public LayerZTest
{
protected:
    virtual void SetUp()
    {
        LayerZTest::SetUp();

        // add request to optional output of neuron weights
        config_.Output(LayerZ::KEY_OUTPUT_WEIGHTS, NAME_OUTPUT_WEIGHTS);
        config_.Output(LayerZ::KEY_OUTPUT_BIAS, NAME_OUTPUT_BIAS);
        to_.Reset(config_);
        to_.IONames(config_);
    }
};

/**
 * @brief test adding request to optional output of neuron weights
 */
TEST_F(LayerZLearnTest, OptionalWeightOutput)
{
    LayerConfig cfg;
    cfg.Params(config_.Params());
    cfg.Input(LayerZ::KEY_INPUT_SPIKES, NAME_INPUT_SPIKES);
    cfg.Output(LayerZ::KEY_OUTPUT_SPIKES, NAME_OUTPUT_SPIKES);

    LayerZ to;
    to.Reset(cfg);
    EXPECT_NO_THROW(to.IONames(cfg)) << "Are all required IO names present?";

    to.Activate(signal_);
    {
        Signal signal_new;
        to.Response(signal_new);
        EXPECT_FALSE(signal_new.Exists(NAME_OUTPUT_MEM_POT));
        EXPECT_FALSE(signal_new.Exists(NAME_OUTPUT_WEIGHTS));
    }

    cfg.Output(LayerZ::KEY_OUTPUT_WEIGHTS, NAME_OUTPUT_WEIGHTS);
    EXPECT_NO_THROW(to.IONames(cfg)) << "Are all required IO names present?";
    to.Activate(signal_);
    {
        Signal signal_new;
        to.Response(signal_new);
        EXPECT_FALSE(signal_new.Exists(NAME_OUTPUT_MEM_POT));
        EXPECT_TRUE(signal_new.Exists(NAME_OUTPUT_WEIGHTS));
        EXPECT_FALSE(signal_new.Exists(NAME_OUTPUT_BIAS));
    }

    EXPECT_NO_THROW(to.IONames(config_)) << "Are all required IO names present?";
    to.Activate(signal_);
    {
        Signal signal_new;
        to.Response(signal_new);
        EXPECT_TRUE(signal_new.Exists(NAME_OUTPUT_MEM_POT));
        EXPECT_TRUE(signal_new.Exists(NAME_OUTPUT_WEIGHTS));
    }
}

/**
 * @brief test adding request to optional output of neuron weights
 */
TEST_F(LayerZLearnTest, OptionalBiasOutput)
{
    LayerConfig cfg;
    cfg.Params(config_.Params());
    cfg.Input(LayerZ::KEY_INPUT_SPIKES, NAME_INPUT_SPIKES);
    cfg.Output(LayerZ::KEY_OUTPUT_SPIKES, NAME_OUTPUT_SPIKES);

    LayerZ to;
    to.Reset(cfg);
    EXPECT_NO_THROW(to.IONames(cfg)) << "Are all required IO names present?";

    to.Activate(signal_);
    {
        Signal signal_new;
        to.Response(signal_new);
        EXPECT_FALSE(signal_new.Exists(NAME_OUTPUT_MEM_POT));
        EXPECT_FALSE(signal_new.Exists(NAME_OUTPUT_WEIGHTS));
        EXPECT_FALSE(signal_new.Exists(NAME_OUTPUT_BIAS));
    }

    cfg.Output(LayerZ::KEY_OUTPUT_WEIGHTS, NAME_OUTPUT_BIAS);
    EXPECT_NO_THROW(to.IONames(cfg)) << "Are all required IO names present?";
    to.Activate(signal_);
    {
        Signal signal_new;
        to.Response(signal_new);
        EXPECT_FALSE(signal_new.Exists(NAME_OUTPUT_MEM_POT));
        EXPECT_FALSE(signal_new.Exists(NAME_OUTPUT_WEIGHTS));
        EXPECT_TRUE(signal_new.Exists(NAME_OUTPUT_BIAS));
    }

    EXPECT_NO_THROW(to.IONames(config_)) << "Are all required IO names present?";
    to.Activate(signal_);
    {
        Signal signal_new;
        to.Response(signal_new);
        EXPECT_TRUE(signal_new.Exists(NAME_OUTPUT_MEM_POT));
        EXPECT_TRUE(signal_new.Exists(NAME_OUTPUT_WEIGHTS));
        EXPECT_TRUE(signal_new.Exists(NAME_OUTPUT_BIAS));
    }
}

/**
 * @brief test we have access to weights before applying anything
 */
TEST_F(LayerZLearnTest, InitialWeights)
{
    const int nb_output_nodes = config_.Params().get<int>(LayerZ::PARAM_NB_OUTPUT_NODES);
    LayerZ to;
    to.Reset(config_);
    to.IONames(config_);
    to.Response(signal_);
    const Mat1f initial_weights = signal_.MostRecentMat(NAME_OUTPUT_WEIGHTS);
    EXPECT_MAT_DIMS_EQ(initial_weights, Size2i(nb_afferents_, nb_output_nodes));
}

/**
 * @brief test underlying neuron bias dimensions
 */
TEST_F(LayerZLearnTest, BiasDims)
{
    const int nb_output_nodes = config_.Params().get<int>(LayerZ::PARAM_NB_OUTPUT_NODES);
    LayerZ to;
    to.Reset(config_);
    to.IONames(config_);
    to.Response(signal_);
    const Mat1f initial_bias = signal_.MostRecentMat(NAME_OUTPUT_BIAS);
    EXPECT_MAT_DIMS_EQ(initial_bias, Size2i(nb_output_nodes, 1));
}

/**
 * @brief Extracting weights from the layer is not always a deep copy.
 * Which is ok, because we don't want this to take up too much memory.
 * But we need to be aware of where the shallow and deep copying occurs.
 *
 * This test demonstrates that we're referencing the same memory block as the layer, a save on memory,
 * however, we need to be careful not to interfere with this sturucture.
 * Given that the weights, are probably going to be used for evaluations and visualizations rather than
 * downstream computation, this is acceptable
 */
TEST_F(LayerZLearnTest, WeightsNotCopied)
{
    to_.Activate(signal_);
    to_.Response(signal_);
    Mat1f w0;

    // get from signal
    Mat1f w1 = signal_.MostRecentMat(NAME_OUTPUT_WEIGHTS);
    w1.copyTo(w0); // back up, deep-copy

    // get form signal one more time
    // any modification on w2 will reflect on w1, because we're only copying
    // the matrix header inside this signal object
    Mat1f w2 = signal_.MostRecentMat(NAME_OUTPUT_WEIGHTS);

    // update signal with layer response and get it a third time
    // any modification on w3 will reflect on w2 and w1, because we're effectivly
    // only copying the header of the matrix that lives inside the header
    // This means we're referencing the same memory block as the layer, a save on memory,
    // however, we need to be careful not to interfere with this sturucture.
    // Given that the weights, are probably going to be used for evaluations and visualizations rather than
    // downstream computation, this is acceptable
    to_.Response(signal_);
    Mat1f w3 = signal_.MostRecentMat(NAME_OUTPUT_WEIGHTS);

    // add increments to the different weight copies
    w1 += 1.f;
    w2 += 2.f;
    w3 += 3.f;

    // check which objects are just references of each other
    // we have to use the NEAR asssertion with low tolerance due to float precision errors
    EXPECT_MAT_NEAR(w0+3.f, w1, 1e-5);
    EXPECT_MAT_NEAR(w0+3.f, w2, 1e-5);
    EXPECT_MAT_NEAR(w1, w2, 1e-7) << "w1 and w2 point to the same underlying data";
    EXPECT_MAT_NEAR(w0+3.f, w3, 1e-5);
}

/**
 * @brief Test layer's weights are static when it's not learning anything.
 */
TEST_F(LayerZLearnTest, Static)
{
    const int N=50;

    Mat w_initial;
    for(int i=0; i<N; i++) {

        to_.Activate(signal_);
        to_.Response(signal_);
        Mat w = signal_.MostRecentMat(NAME_OUTPUT_WEIGHTS);

        if(i == 0) {

            w.copyTo(w_initial);
        }
        else {

            EXPECT_MAT_EQ(w, w_initial) << "\'u\' weights are not static inspite of no learning taking place.";
        }
    }
}

/**
 * @brief Apply learning algorithm while disabling any spiking
 * We expect the weights to remain unchanged and the bias to decrease
 */
TEST_F(LayerZLearnTest, Learn_NoFire)
{
    PTree params = config_.Params();
    params.put(LayerZ::PARAM_WTA_FREQ, 0.f);
    config_.Params(params);
    to_.Reset(config_);
    to_.IONames(config_);

    to_.Response(signal_);
    const Mat1f initial_weights = signal_.MostRecentMat(NAME_OUTPUT_WEIGHTS).clone();

    for(int i=0; i<50; i++) {

        signal_.Append(NAME_INPUT_SPIKES, Mat1f::ones(1, nb_afferents_));

        Mat1f bias_prev = signal_.MostRecentMat(NAME_OUTPUT_BIAS).clone();

        to_.Activate(signal_);
        to_.Learn();
        to_.Response(signal_);
        Mat1f weights = signal_.MostRecentMat(NAME_OUTPUT_WEIGHTS).clone();
        Mat1f bias = signal_.MostRecentMat(NAME_OUTPUT_BIAS).clone();

        EXPECT_MAT_EQ(initial_weights, weights) << "weights changed inspite of disabling spiking";

        EXPECT_MAT_DIMS_EQ(bias_prev, bias) << "bias dimensions changed";
        EXPECT_MAT_LT(bias, bias_prev) << "Bias not decaying";
    }
}

/**
 * @brief Test learning with afferent input that spikes in alternating fashion
 * if i spikes i+1 does not spike and vice versa
 * Then check impact of learning on weights
 * This test requires the WTA circuit's firing rate to be adequately high.
 * It should spike at least once during the test's iterations.
 */
TEST_F(LayerZLearnTest, Learn)
{
    FakeEvidence stimuli(nb_afferents_); // afferents alternate in spiking

    bool checked = false;

    for(int i=0; i<50; i++) {

        to_.Response(signal_);
        Mat1f bias_prev = signal_.MostRecentMat(NAME_OUTPUT_BIAS).clone();
        Mat1f weights_prev = signal_.MostRecentMat(NAME_OUTPUT_WEIGHTS).clone();

        signal_.Append(NAME_INPUT_SPIKES, stimuli.next(0));

        to_.Activate(signal_);
        to_.Learn();
        to_.Response(signal_);

        Mat1f weights = signal_.MostRecentMat(NAME_OUTPUT_WEIGHTS).clone();
        Mat1f bias = signal_.MostRecentMat(NAME_OUTPUT_BIAS).clone();

        // who spiked?
        Mat spikes_out = signal_.MostRecentMat(NAME_OUTPUT_SPIKES);
        int spiking_neuron_index;

        // Spiking in the WTA circuit is probabilistic, so we want to go through the checks at least once.
        if( sem::find_first_of(spikes_out > 0, static_cast<uchar>(255), spiking_neuron_index) ) {

            ASSERT_GE( spiking_neuron_index, 0 );

            EXPECT_FALSE( Equal(weights_prev.row(spiking_neuron_index), weights.row(spiking_neuron_index)) );

            for(int i=0; i<weights_prev.cols; i+=2) {

                EXPECT_GT(weights_prev(spiking_neuron_index, i+1), weights(spiking_neuron_index, i+1)) << "Weight for non-spiking input potentiating.";
                EXPECT_LT(weights_prev(spiking_neuron_index, i), weights(spiking_neuron_index, i)) << "Weight for spiking input decaying.";
            }
            EXPECT_LT(bias_prev(spiking_neuron_index), bias(spiking_neuron_index)) << "Bias not increasing for spiking neuron.";

            checked = true;
        }
    }

    // sanity check that we performed the assertions.
    ASSERT_TRUE(checked) << "Assertions were not performed, the WTA circuits never spiked.";
}

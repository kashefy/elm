#include "layers/layer_z.h"

#include "core/exception.h"
#include "core/ptree_utils.h"
#include "ts/ts.h"

using namespace std;
using namespace cv;
using namespace sem;

/**
 * @brief class for testing layer Z initialization, the main, SEM learning algorithm
 */
class LayerZInitTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        nb_afferents_ = 50;
        PTree params;
        params.put(LayerZ::PARAM_NB_AFFERENTS, nb_afferents_);
        params.put(LayerZ::PARAM_NB_OUTPUT_NODES, 10);
        config_.Params(params);

        // IO
        config_.Input(LayerZ::KEY_INPUT_SPIKES, NAME_INPUT_SPIKES);

        config_.Output(LayerZ::KEY_OUTPUT_SPIKES, NAME_OUTPUT_SPIKES);
        config_.Output(LayerZ::KEY_OUTPUT_MEMBRANE_POT, NAME_OUTPUT_MEM_POT);
    }

    // members
    LayerZ to_;             ///< test object
    LayerConfig config_;    ///< default layer configuration

    int nb_afferents_;
    // name of keys in signal
    static const string NAME_INPUT_SPIKES;      ///< no. of afferent spikes
    static const string NAME_OUTPUT_SPIKES;     ///< no. of output spikes/size of output layer
    static const string NAME_OUTPUT_MEM_POT;    ///< membrane potential
};
// initialize static members
const string LayerZInitTest::NAME_INPUT_SPIKES  = "in";
const string LayerZInitTest::NAME_OUTPUT_SPIKES = "out";
const string LayerZInitTest::NAME_OUTPUT_MEM_POT = "mem_pot";

/**
 * @brief test validation of missing params
 */
TEST_F(LayerZInitTest, MissingParams)
{
    // remove key to output nodes
    PTree params = config_.Params();
    params.erase(LayerZ::PARAM_NB_OUTPUT_NODES);
    config_.Params(params);

    LayerZ to;
    EXPECT_THROW(to.Reset(config_), std::exception);
}

/**
 * @brief test parameter validation
 */
TEST_F(LayerZInitTest, InvalidParams)
{
    {
        PTree params = config_.Params();
        params.put(LayerZ::PARAM_DELTA_T, -0.1f);
        config_.Params(params);

        LayerZ to;
        EXPECT_THROW(to.Reset(config_), ExceptionValueError);
    }
    {
        PTree params = config_.Params();
        params.put(LayerZ::PARAM_DELTA_T, 0.f);
        config_.Params(params);

        LayerZ to;
        EXPECT_THROW(to.Reset(config_), ExceptionValueError);
    }
    {
        PTree params = config_.Params();
        params.put(LayerZ::PARAM_DELTA_T, -0.f);
        config_.Params(params);

        LayerZ to;
        EXPECT_THROW(to.Reset(config_), ExceptionValueError);
    }
    {
        PTree params = config_.Params();
        params.put(LayerZ::PARAM_LEN_HISTORY, -3);
        config_.Params(params);

        LayerZ to;
        EXPECT_THROW(to.Reset(config_), ExceptionValueError);
    }
    {
        PTree params = config_.Params();
        params.put(LayerZ::PARAM_LEN_HISTORY, 0);
        config_.Params(params);

        LayerZ to;
        EXPECT_THROW(to.Reset(config_), ExceptionValueError);
    }
    {
        PTree params = config_.Params();
        params.put(LayerZ::PARAM_NB_AFFERENTS, 0);
        config_.Params(params);

        LayerZ to;
        EXPECT_THROW(to.Reset(config_), ExceptionValueError);
    }
    {
        PTree params = config_.Params();
        params.put(LayerZ::PARAM_NB_AFFERENTS, -3);
        config_.Params(params);

        LayerZ to;
        EXPECT_THROW(to.Reset(config_), ExceptionValueError);
    }
    {
        PTree params = config_.Params();
        params.put(LayerZ::PARAM_WTA_FREQ, -1);
        config_.Params(params);

        LayerZ to;
        EXPECT_THROW(to.Reset(config_), ExceptionValueError);
    }
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

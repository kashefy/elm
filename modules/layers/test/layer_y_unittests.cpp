#include "layers/layer_y.h"

#include "core/exception.h"
#include "core/layerconfig.h"
#include "core/signal.h"
#include "layers/layerfactory.h"
#include "ts/ts.h"

using namespace sem;
using boost::property_tree::ptree_bad_path;

namespace {

const std::string NAME_STIMULUS = "in";
const std::string NAME_SPIKES   = "out";

/**
 * @brief class for testing layer Y initializations
 */
class LayerYInitTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        params_.put(LayerY::PARAM_FREQ, 40.f);
        params_.put(LayerY::PARAM_DELTA_T_MSEC, 1.f);

        config_ = LayerConfig();
        config_.Params(params_);
    }

    virtual void TearDown()
    {
        params_.clear();
    }

    PTree params_;
    LayerConfig config_;
};

TEST_F(LayerYInitTest, ParamsValid)
{
    EXPECT_NO_THROW(LayerY(config_));
}

TEST_F(LayerYInitTest, ParamsInValid)
{
    {
        PTree p;
        p.put(LayerY::PARAM_FREQ, -1.f);
        p.put(LayerY::PARAM_DELTA_T_MSEC, 1.f);
        LayerConfig c;
        c.Params(p);
        EXPECT_THROW(LayerY to(c), ExceptionValueError);
    }
    {
        PTree p;
        p.put(LayerY::PARAM_FREQ, 1.f);
        p.put(LayerY::PARAM_DELTA_T_MSEC, -1.f);
        LayerConfig c;
        c.Params(p);
        EXPECT_THROW(LayerY to(c), ExceptionValueError);
    }
    {
        PTree p;
        p.put(LayerY::PARAM_FREQ, 1.f);
        p.put(LayerY::PARAM_DELTA_T_MSEC, 0.f);
        LayerConfig c;
        c.Params(p);
        EXPECT_THROW(LayerY to(c), ExceptionValueError);
    }
}

TEST_F(LayerYInitTest, ParamsMissing)
{
    {
        LayerY to;
        EXPECT_THROW(to.Reset(LayerConfig());, ptree_bad_path);
    }
    {
        PTree p;
        p.put(LayerY::PARAM_DELTA_T_MSEC, 1.f);
        LayerConfig cfg;
        cfg.Params(p);
        EXPECT_THROW(LayerY to(cfg), ptree_bad_path);
    }
    {
        PTree p;
        p.put(LayerY::PARAM_FREQ, 1.f);
        LayerConfig cfg;
        cfg.Params(p);
        EXPECT_THROW(LayerY to(cfg), ptree_bad_path);
    }
}

TEST_F(LayerYInitTest, IONamesValid)
{
    LayerY to;
    to.Reset(config_);
    LayerIONames io;
    io.Input(LayerY::KEY_INPUT_STIMULUS, NAME_STIMULUS);
    io.Output(LayerY::KEY_OUTPUT_SPIKES, NAME_SPIKES);
    EXPECT_NO_THROW(to.IONames(io));
}

TEST_F(LayerYInitTest, IONamesMissing)
{
    {
        LayerY to;
        to.Reset(config_);
        EXPECT_THROW(to.IONames(LayerIONames()), ExceptionKeyError);
    }
    {
        LayerY to;
        to.Reset(config_);
        LayerIONames io;
        io.Input(LayerY::KEY_INPUT_STIMULUS, NAME_STIMULUS);
        EXPECT_THROW(to.IONames(io), ExceptionKeyError);
    }
    {
        LayerY to;
        to.Reset(config_);
        LayerIONames io;
        io.Output(LayerY::KEY_OUTPUT_SPIKES, NAME_SPIKES);
        EXPECT_THROW(to.IONames(io), ExceptionKeyError);
    }
}

TEST_F(LayerYInitTest, Create)
{
    LayerIONames io;
    io.Input(LayerY::KEY_INPUT_STIMULUS, NAME_STIMULUS);
    io.Output(LayerY::KEY_OUTPUT_SPIKES, NAME_SPIKES);
    std::shared_ptr<base_Layer> ptr = LayerFactory::CreateLayerPtrShared(
                "LayerY",
                config_,
                io);
    EXPECT_TRUE(bool(ptr));
}

class LayerYTest : public LayerYInitTest
{
protected:
    virtual void SetUp()
    {
        LayerYInitTest::SetUp();

        io_.Input(LayerY::KEY_INPUT_STIMULUS, NAME_STIMULUS);
        io_.Output(LayerY::KEY_OUTPUT_SPIKES, NAME_SPIKES);

        to_ = LayerFactory::CreateLayerPtrShared("LayerY", config_, io_);
    }

    LayerIONames io_;
    std::shared_ptr<base_Layer> to_;    ///< test object
};

TEST_F(LayerYTest, Apply)
{
    Signal s;
    s.Append(NAME_STIMULUS, cv::Mat1f::zeros(1, 1));

    to_->Stimulus(s);
    to_->Apply();

    EXPECT_FALSE(s.Exists(NAME_SPIKES));

    to_->Response(s);

    EXPECT_TRUE(s.Exists(NAME_SPIKES)) << "Response missing";
}

/**
 * @brief test layer application with all zero stimulus
 */
TEST_F(LayerYTest, Zero)
{
    const int N = 1000;
    for(int i=0; i<N; i++) {

        cv::Mat1f x = cv::Mat1f::zeros(1, 1);

        Signal s;
        s.Append(NAME_STIMULUS, x.clone());

        to_->Stimulus(s);
        to_->Apply();
        to_->Response(s);

        ASSERT_TRUE(s.Exists(NAME_SPIKES)) << "Response missing";

        EXPECT_MAT_EQ(s.MostRecent(NAME_SPIKES), x);
    }
}

/**
 * @brief test with stimulus containing invalid values
 */
TEST_F(LayerYTest, Invalid)
{
    const int N=5;
    float data[N] = {-11.f, -2.f, 2.f, 11.f, 100.f};
    cv::Mat1f x(1, N, data);

    for(int i=0; i<N; i++) {

        Signal s;
        s.Append(NAME_STIMULUS, x.col(i));

        to_->Stimulus(s);
        EXPECT_THROW(to_->Apply(), ExceptionNotImpl);
    }
}

TEST_F(LayerYTest, Freq_Inf)
{
    const int N = 100;
    PTree p = config_.Params();
    p.put(LayerY::PARAM_FREQ, 1000.f);
    config_.Params(p);
    to_->Reset(config_);

    for(int i=0; i<N; i++) {

        cv::Mat1f st(1, 1, i%2);
        Signal s;
        s.Append(NAME_STIMULUS, st.clone());

        to_->Stimulus(s);
        to_->Apply();
        to_->Response(s);

        EXPECT_MAT_DIMS_EQ(s.MostRecent(NAME_SPIKES), st);
    }
}

TEST_F(LayerYTest, Freq_Zero)
{
    const int N = 100;
    PTree p = config_.Params();
    p.put(LayerY::PARAM_FREQ, 0.f);
    config_.Params(p);
    to_->Reset(config_);

    for(int i=0; i<N; i++) {

        cv::Mat1f st(1, 1, i%2);
        Signal s;
        s.Append(NAME_STIMULUS, st.clone());

        to_->Stimulus(s);
        to_->Apply();
        to_->Response(s);

        EXPECT_MAT_DIMS_EQ(s.MostRecent(NAME_SPIKES), cv::Mat1f::zeros(st.size()));
    }
}

} // annonymous namespace

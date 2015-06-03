/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/layers/layer_y.h"

#include "elm/core/exception.h"
#include "elm/core/layerconfig.h"
#include "elm/core/signal.h"
#include "elm/layers/layerfactory.h"
#include "elm/ts/layer_assertions.h"
#include "elm/ts/mat_assertions.h"

using namespace elm;
using boost::property_tree::ptree_bad_path;

namespace {

ELM_INSTANTIATE_LAYER_TYPED_TEST_CASE_P(LayerY);

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
        LayerY to;
        EXPECT_THROW(to.Reset(c), ExceptionValueError);
        EXPECT_THROW(to.Reconfigure(c), ExceptionValueError);
    }
    {
        PTree p;
        p.put(LayerY::PARAM_FREQ, 1.f);
        p.put(LayerY::PARAM_DELTA_T_MSEC, -1.f);
        LayerConfig c;
        c.Params(p);
        LayerY to;
        EXPECT_THROW(to.Reset(c), ExceptionValueError);
        EXPECT_THROW(to.Reconfigure(c), ExceptionValueError);
    }
    {
        PTree p;
        p.put(LayerY::PARAM_FREQ, 1.f);
        p.put(LayerY::PARAM_DELTA_T_MSEC, 0.f);
        LayerConfig c;
        c.Params(p);
        LayerY to;
        EXPECT_THROW(to.Reset(c), ExceptionValueError);
        EXPECT_THROW(to.Reconfigure(c), ExceptionValueError);
    }
}

TEST_F(LayerYInitTest, ParamsMissing_Reset)
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
        LayerY to;
        EXPECT_THROW(to.Reset(cfg), ptree_bad_path);
        EXPECT_THROW(to.Reconfigure(cfg), ptree_bad_path);
    }
    {
        PTree p;
        p.put(LayerY::PARAM_FREQ, 1.f);
        LayerConfig cfg;
        cfg.Params(p);
        LayerY to;
        EXPECT_THROW(to.Reset(cfg), ptree_bad_path);
        EXPECT_THROW(to.Reconfigure(cfg), ptree_bad_path);
    }
}

TEST_F(LayerYInitTest, Create)
{
    LayerIONames io;
    io.Input(LayerY::KEY_INPUT_STIMULUS, NAME_STIMULUS);
    io.Output(LayerY::KEY_OUTPUT_RESPONSE, NAME_SPIKES);
    LayerShared ptr = LayerFactory::CreateShared(
                "LayerY",
                config_,
                io);
    EXPECT_TRUE(bool(ptr));
}

TEST_F(LayerYInitTest, Extra_keys)
{
    LayerIONames io;
    io.Input(LayerY::KEY_INPUT_STIMULUS, NAME_STIMULUS);
    io.Output(LayerY::KEY_OUTPUT_RESPONSE, NAME_SPIKES);
    PTree p = config_.Params();
    p.put("foo", 1);
    config_.Params(p);
    EXPECT_THROW(LayerFactory::CreateShared(
                     "LayerY",
                     config_,
                     io), ExceptionKeyError);
}

class LayerYTest : public LayerYInitTest
{
protected:
    virtual void SetUp()
    {
        LayerYInitTest::SetUp();

        io_.Input(LayerY::KEY_INPUT_STIMULUS, NAME_STIMULUS);
        io_.Output(LayerY::KEY_OUTPUT_RESPONSE, NAME_SPIKES);

        to_ = LayerFactory::CreateShared("LayerY", config_, io_);
    }

    LayerIONames io_;
    LayerShared to_;    ///< test object
};

TEST_F(LayerYTest, Clear)
{
    EXPECT_NO_THROW(to_->Clear());
    EXPECT_NO_THROW(to_->Clear());
}

TEST_F(LayerYTest, Activate)
{
    Signal s;
    s.Append(NAME_STIMULUS, cv::Mat1f::zeros(1, 1));

    to_->Activate(s);

    EXPECT_FALSE(s.Exists(NAME_SPIKES));

    to_->Response(s);

    EXPECT_TRUE(s.Exists(NAME_SPIKES)) << "Response missing";
}

/**
 * @brief test layer activation with all zero stimulus
 */
TEST_F(LayerYTest, Zero)
{
    const int N = 1000;
    for(int i=0; i<N; i++) {

        cv::Mat1f x = cv::Mat1f::zeros(1, 1);

        Signal s;
        s.Append(NAME_STIMULUS, x.clone());

        to_->Activate(s);
        to_->Response(s);

        ASSERT_TRUE(s.Exists(NAME_SPIKES)) << "Response missing";

        EXPECT_MAT_EQ(s.MostRecentMat1f(NAME_SPIKES), x);
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

        EXPECT_THROW(to_->Activate(s), ExceptionNotImpl);
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

        to_->Activate(s);
        to_->Response(s);

        EXPECT_MAT_DIMS_EQ(s.MostRecentMat1f(NAME_SPIKES), st);
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

        to_->Activate(s);
        to_->Response(s);

        EXPECT_MAT_DIMS_EQ(s.MostRecentMat1f(NAME_SPIKES), cv::Mat1f::zeros(st.size()));
    }
}

/**
 * @brief Activate layer with empty stimuli
 */
TEST_F(LayerYTest, Activate_Empty)
{
    cv::Mat1f st;
    Signal s;
    s.Append(NAME_STIMULUS, st.clone());

    to_->Activate(s);
    to_->Response(s);

    EXPECT_TRUE(s.MostRecentMat1f(NAME_SPIKES).empty());
}

TEST_F(LayerYTest, Response_Dims)
{
    const int ROWS=7;
    const int COLS=7;

    Signal s;

    for(int r=1; r<ROWS; r++) {

        for(int c=1; c<COLS; c++) {

            cv::Mat1f st(r, c);
            randn(st, 0.f, 10.f);
            st.setTo(0.f, st <= 0.5f);
            st.setTo(1.f, st > 0.5f);

            s.Append(NAME_STIMULUS, st.clone());

            to_->Activate(s);
            to_->Response(s);

            EXPECT_MAT_DIMS_EQ(s.MostRecentMat1f(NAME_SPIKES), st.size());
        }

        s.Clear();
    }

}

} // annonymous namespace

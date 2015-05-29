/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/layers/layergraph_impl.h"

#include "gtest/gtest.h"

#include "elm/core/debug_utils.h"
#include "elm/core/exception.h"
#include "elm/core/inputname.h"
#include "elm/core/layerconfig.h"
#include "elm/core/signal.h"
#include "elm/layers/layers_interim/base_featuretransformationlayer.h"
#include "elm/ts/container.h"
#include "elm/ts/mat_assertions.h"

using cv::Mat1f;
using namespace elm;

namespace {

class LayerA : public base_FeatureTransformationLayer
{
public:
    static const std::string PARAM_CAPITAL;     ///< parameter for capital letters
    static const std::string PARAM_DUPLICATE;   ///< parameter for duplicating last row

    void Clear() {}

    void Reconfigure(const LayerConfig &config) {

        do_capital_ = config.Params().get<bool>(PARAM_CAPITAL, false);
        do_duplicate_ = config.Params().get<bool>(PARAM_DUPLICATE, false);
    }

    void Activate(const Signal &signal) {

        m_ = signal.MostRecentMat1f(name_input_).clone();

        char c = do_capital_ ? 'A' : 'a';
        m_.push_back(Mat1f(1, 1, static_cast<float>(c)));

        if(do_duplicate_) {

            m_.push_back(m_.row(m_.rows-1));
        }
    }

    bool do_capital_;    ///< flag for capital letters
    bool do_duplicate_;  ///< flag for duplicating last row
};
const std::string LayerA::PARAM_CAPITAL      = "capital";
const std::string LayerA::PARAM_DUPLICATE    = "dup";

class LayerB : public base_FeatureTransformationLayer
{
public:
    static const std::string PARAM_CAPITAL;    ///< parameter for capital letters

    void Clear() {}

    void Reconfigure(const LayerConfig &config) {

        do_capital_ = config.Params().get<bool>(PARAM_CAPITAL, false);
    }

    void Activate(const Signal &signal) {

        m_ = signal.MostRecentMat1f(name_input_).clone();

        char c = do_capital_ ? 'B' : 'b';
        m_.push_back(Mat1f(1, 1, static_cast<float>(c)));
    }

    bool do_capital_;
};
const std::string LayerB::PARAM_CAPITAL = "capital";

class LayerC : public base_FeatureTransformationLayer
{
public:
    void Clear() {}

    void Reconfigure(const LayerConfig &config) {}

    void Activate(const Signal &signal) {

        m_ = signal.MostRecentMat1f(name_input_).clone();
        m_.push_back(Mat1f(1, 1, static_cast<float>('c')));
    }
};

class LayerD : public base_FeatureTransformationLayer
{
public:
    void Clear() {}

    void Reconfigure(const LayerConfig &config) {}

    void Activate(const Signal &signal) {

        m_ = signal.MostRecentMat1f(name_input_).clone();
        m_.push_back(Mat1f(1, 1, static_cast<float>('d')));
    }
};

class LayerGraphImplTest : public ::testing::Test
{
};

TEST_F(LayerGraphImplTest, ClearActive_empty) {

    EXPECT_NO_THROW(LayerGraph_Impl().ClearActive());
}

TEST_F(LayerGraphImplTest, ClearActive) {

    LayerGraph_Impl to;

    std::shared_ptr<base_Layer> a(new LayerA);

    LayerConfig cfg;
    PTree p;
    p.put("pa", "pa1");
    LayerIONames io;
    io.Input(LayerA::KEY_INPUT_STIMULUS, "ina");
    io.Output(LayerA::KEY_OUTPUT_RESPONSE, "outa");
    to.Add("a", a, cfg, io);

    ASSERT_TRUE(to.Outputs().empty());
    to.AddOutput("outa");

    ASSERT_FALSE(to.Outputs().empty());

    to.ClearActive();

    EXPECT_TRUE(to.Outputs().empty());
}

TEST_F(LayerGraphImplTest, Outputs_empty) {

    EXPECT_TRUE(LayerGraph_Impl().Outputs().empty());
}

TEST_F(LayerGraphImplTest, Outputs_single) {

    LayerGraph_Impl to;

    std::shared_ptr<base_Layer> a(new LayerA);

    LayerConfig cfg;
    PTree p;
    p.put("pa", "pa1");
    LayerIONames io;
    io.Input(LayerA::KEY_INPUT_STIMULUS, "ina");
    io.Output(LayerA::KEY_OUTPUT_RESPONSE, "outa");
    to.Add("a", a, cfg, io);

    EXPECT_TRUE(to.Outputs().empty());
    to.AddOutput("outa");

    SetS outputs = to.Outputs();
    ASSERT_FALSE(outputs.empty());

    EXPECT_EQ(SetS({"outa"}), to.Outputs()) << "Unexpected outputs";
    EXPECT_NE(SetS({"ina"}), to.Outputs()) << "Outputs confused with inputs.";
}

TEST_F(LayerGraphImplTest, Inputs_empty) {

    EXPECT_TRUE(LayerGraph_Impl().Inputs().empty());
}

TEST_F(LayerGraphImplTest, Inputs_single) {

    LayerGraph_Impl to;

    std::shared_ptr<base_Layer> a(new LayerA);

    LayerConfig cfg;
    PTree p;
    p.put("pa", "pa1");
    LayerIONames io;
    io.Input(LayerA::KEY_INPUT_STIMULUS, "ina");
    io.Output(LayerA::KEY_OUTPUT_RESPONSE, "outa");
    to.Add("a", a, cfg, io);

    EXPECT_TRUE(to.Inputs().empty());
    to.AddOutput("outa");

    SetS inputs = to.Inputs();
    ASSERT_FALSE(inputs.empty());

    EXPECT_EQ(SetS({"ina"}), to.Inputs()) << "Unexpected inputs";
    EXPECT_NE(SetS({"outa"}), to.Inputs()) << "Inputs confused with outputs.";
}

TEST_F(LayerGraphImplTest, Inputs_chained) {

    LayerGraph_Impl to;

    std::shared_ptr<base_Layer> a(new LayerA);
    std::shared_ptr<base_Layer> b(new LayerB);

    {
        LayerConfig cfg;
        PTree p;
        p.put("pa", "pa1");
        LayerIONames io;
        io.Input(LayerA::KEY_INPUT_STIMULUS, "ina");
        io.Output(LayerA::KEY_OUTPUT_RESPONSE, "outa");
        to.Add("a", a, cfg, io);
    }
    {
        LayerConfig cfg;
        PTree p;
        p.put("pb", "pb1");
        LayerIONames io;
        io.Input(LayerB::KEY_INPUT_STIMULUS, "outa");
        io.Output(LayerB::KEY_OUTPUT_RESPONSE, "outb");
        to.Add("b", b, cfg, io);
    }

    EXPECT_TRUE(to.Inputs().empty());
    to.AddOutput("outb");

    SetS inputs = to.Inputs();
    ASSERT_FALSE(inputs.empty());
    EXPECT_SIZE(1, to.Inputs());

    EXPECT_EQ(SetS({"ina"}), to.Inputs()) << "Unexpected inputs";
    EXPECT_NE(SetS({"outa"}), to.Inputs()) << "Inputs confused with outputs.";
}

TEST_F(LayerGraphImplTest, Inputs_multiple) {

    LayerGraph_Impl to;

    std::shared_ptr<base_Layer> a(new LayerA);
    std::shared_ptr<base_Layer> b(new LayerB);

    {
        LayerConfig cfg;
        PTree p;
        p.put("pa", "pa1");
        LayerIONames io;
        io.Input(LayerA::KEY_INPUT_STIMULUS, "ina");
        io.Output(LayerA::KEY_OUTPUT_RESPONSE, "outa");
        to.Add("a", a, cfg, io);
    }
    {
        LayerConfig cfg;
        PTree p;
        p.put("pb", "pb1");
        LayerIONames io;
        io.Input(LayerB::KEY_INPUT_STIMULUS, "inb");
        io.Output(LayerB::KEY_OUTPUT_RESPONSE, "outb");
        to.Add("b", b, cfg, io);
    }

    EXPECT_TRUE(to.Inputs().empty());
    to.AddOutput("outa");
    to.AddOutput("outb");

    SetS inputs = to.Inputs();
    ASSERT_FALSE(inputs.empty());
    EXPECT_SIZE(2, to.Inputs());

    EXPECT_EQ(SetS({"ina", "inb"}), to.Inputs()) << "Unexpected inputs";
    EXPECT_NE(SetS({"outa", "outb"}), to.Inputs()) << "Inputs confused with outputs.";
    EXPECT_NE(to.Outputs(), to.Inputs()) << "Inputs confused with outputs.";
}

/**
 * @brief add another layer to the graph with its own input but don't activate it
 */
TEST_F(LayerGraphImplTest, Inputs_single_active_only) {

    LayerGraph_Impl to;

    std::shared_ptr<base_Layer> a(new LayerA);
    std::shared_ptr<base_Layer> b(new LayerB);

    {
        LayerConfig cfg;
        PTree p;
        p.put("pa", "pa1");
        LayerIONames io;
        io.Input(LayerA::KEY_INPUT_STIMULUS, "ina");
        io.Output(LayerA::KEY_OUTPUT_RESPONSE, "outa");
        to.Add("a", a, cfg, io);
    }
    {
        LayerConfig cfg;
        PTree p;
        p.put("pb", "pb1");
        LayerIONames io;
        io.Input(LayerB::KEY_INPUT_STIMULUS, "inb");
        io.Output(LayerB::KEY_OUTPUT_RESPONSE, "outb");
        to.Add("b", b, cfg, io);
    }

    EXPECT_TRUE(to.Inputs().empty());
    to.AddOutput("outa");

    SetS inputs = to.Inputs();
    ASSERT_FALSE(inputs.empty());
    EXPECT_SIZE(1, to.Inputs());

    EXPECT_EQ(SetS({"ina"}), to.Inputs()) << "Unexpected inputs";
    EXPECT_NE(SetS({"outa"}), to.Inputs()) << "Inputs confused with outputs.";
    EXPECT_NE(to.Outputs(), to.Inputs()) << "Inputs confused with outputs.";
}

TEST_F(LayerGraphImplTest, AddOutput) {

    LayerGraph_Impl to;

    std::shared_ptr<base_Layer> a(new LayerA);
    std::shared_ptr<base_Layer> b(new LayerB);
    std::shared_ptr<base_Layer> c(new LayerC);
    std::shared_ptr<base_Layer> d(new LayerD);

    {
        LayerConfig cfg;
        PTree p;
        p.put("pb", "pb1");
        LayerIONames io;
        io.Input(LayerC::KEY_INPUT_STIMULUS, "outa");
        io.Output(LayerC::KEY_OUTPUT_RESPONSE, "outb");
        to.Add("b", b, cfg, io);
    }
    {
        LayerConfig cfg;
        PTree p;
        p.put("pc", "pc1");
        LayerIONames io;
        io.Input(LayerA::KEY_INPUT_STIMULUS, "outb");
        io.Output(LayerA::KEY_OUTPUT_RESPONSE, "outc");
        to.Add("c", c, cfg, io);
    }
    {
        LayerConfig cfg;
        PTree p;
        p.put("pa", "pa1");
        LayerIONames io;
        io.Input(LayerA::KEY_INPUT_STIMULUS, "ina");
        io.Output(LayerA::KEY_OUTPUT_RESPONSE, "outa");
        to.Add("a", a, cfg, io);
    }
    {
        LayerConfig cfg;
        PTree p;
        p.put("pd", "pd1");
        LayerIONames io;
        io.Input(LayerD::KEY_INPUT_STIMULUS, "ind");
        io.Output(LayerD::KEY_OUTPUT_RESPONSE, "outd");
        to.Add("d", d, cfg, io);
    }

    to.AddOutput("outc");

    EXPECT_EQ(SetS({"outc", "outb", "outa"}), to.Outputs()) << "Unexpected outputs";

    to.ClearActive();

    EXPECT_TRUE(to.Outputs().empty()) << "Outputs inspite of clearing active layers.";

    to.AddOutput("outd");

    EXPECT_EQ(SetS({"outd"}), to.Outputs()) << "Unexpected outputs";

    to.ClearActive();

    to.AddOutput("outb");

    EXPECT_EQ(SetS({"outb", "outa"}), to.Outputs()) << "Unexpected outputs";

    to.AddOutput("outd"); // add second output without clearing

    EXPECT_EQ(SetS({"outb", "outa", "outd"}), to.Outputs()) << "Unexpected outputs";

    to.ClearActive();
}

TEST_F(LayerGraphImplTest, AddOutput_invalid) {

    LayerGraph_Impl to;

    EXPECT_THROW(to.AddOutput("outa"), ExceptionKeyError);

    std::shared_ptr<base_Layer> a(new LayerA);

    {
        LayerConfig cfg;
        PTree p;
        p.put("pb", "pb1");
        LayerIONames io;
        io.Input(LayerA::KEY_INPUT_STIMULUS, "outa");
        io.Output(LayerA::KEY_INPUT_STIMULUS, "outb");
        to.Add("b", a, cfg, io);
    }

    EXPECT_THROW(to.AddOutput("out"), ExceptionKeyError);
    EXPECT_THROW(to.AddOutput("outa"), ExceptionKeyError);
}

TEST_F(LayerGraphImplTest, Get_num_layers) {

    LayerGraph_Impl to;

    size_t expected_sz = 0;
    EXPECT_EQ(expected_sz++, to.num_layers());

    std::shared_ptr<base_Layer> a(new LayerA);
    std::shared_ptr<base_Layer> b(new LayerB);
    std::shared_ptr<base_Layer> c(new LayerC);
    std::shared_ptr<base_Layer> d(new LayerD);

    {
        LayerConfig cfg;
        PTree p;
        p.put("pb", "pb1");
        LayerIONames io;
        io.Input(LayerC::KEY_INPUT_STIMULUS, "outa");
        io.Output(LayerC::KEY_OUTPUT_RESPONSE, "outb");
        to.Add("b", b, cfg, io);

        EXPECT_EQ(expected_sz++, to.num_layers());
    }
    {
        LayerConfig cfg;
        PTree p;
        p.put("pc", "pc1");
        LayerIONames io;
        io.Input(LayerA::KEY_INPUT_STIMULUS, "outb");
        io.Output(LayerA::KEY_OUTPUT_RESPONSE, "outc");
        to.Add("c", c, cfg, io);

        EXPECT_EQ(expected_sz++, to.num_layers());
    }
    {
        LayerConfig cfg;
        PTree p;
        p.put("pa", "pa1");
        LayerIONames io;
        io.Input(LayerA::KEY_INPUT_STIMULUS, "ina");
        io.Output(LayerA::KEY_OUTPUT_RESPONSE, "outa");
        to.Add("a", a, cfg, io);

        EXPECT_EQ(expected_sz++, to.num_layers());
    }
    {
        LayerConfig cfg;
        PTree p;
        p.put("pd", "pd1");
        LayerIONames io;
        io.Input(LayerD::KEY_INPUT_STIMULUS, "ind");
        io.Output(LayerD::KEY_OUTPUT_RESPONSE, "outd");
        to.Add("d", d, cfg, io);

        EXPECT_EQ(expected_sz++, to.num_layers());
    }
}

TEST_F(LayerGraphImplTest, GenVtxColor_name) {

    LayerGraph_Impl to;
    VtxColor a = to.genVtxColor("a", LayerConfig(), LayerIONames());

    EXPECT_EQ(a, to.genVtxColor("a", LayerConfig(), LayerIONames()));
    EXPECT_NE(a, to.genVtxColor("a ", LayerConfig(), LayerIONames()));
    EXPECT_NE(a, to.genVtxColor("a1", LayerConfig(), LayerIONames()));
    EXPECT_NE(a, to.genVtxColor("b", LayerConfig(), LayerIONames()));
}

TEST_F(LayerGraphImplTest, GenVtxColor_cfg) {

    LayerGraph_Impl to;
    LayerConfig cfg;
    VtxColor a = to.genVtxColor("a", cfg, LayerIONames());

    EXPECT_EQ(a, to.genVtxColor("a", cfg, LayerIONames()));

    PTree p;
    p.put("foo", 1);
    cfg.Params(p);

    VtxColor a1 = to.genVtxColor("a", cfg, LayerIONames());

    EXPECT_NE(a, a1);

    VtxColor a2 = to.genVtxColor("a", cfg, LayerIONames());

    EXPECT_EQ(a1, a2);

    p = cfg.Params();
    p.put("foo", 2);
    cfg.Params(p);

    a2 = to.genVtxColor("a", cfg, LayerIONames());
    EXPECT_NE(a1, a2);

    p = cfg.Params();
    p.put("bar", 1);
    cfg.Params(p);

    a2 = to.genVtxColor("a", cfg, LayerIONames());
    EXPECT_NE(a1, a2);

    p = PTree();
    p.put("bar", 1);
    cfg.Params(p);

    a2 = to.genVtxColor("a", cfg, LayerIONames());
    EXPECT_NE(a1, a2);

    p = cfg.Params();
    p.put("bar", 3);
    cfg.Params(p);

    a2 = to.genVtxColor("a", cfg, LayerIONames());
    EXPECT_NE(a1, a2);

    p = PTree();
    p.put("foo", 1);
    cfg.Params(p);
    a2 = to.genVtxColor("a", cfg, LayerIONames());
    EXPECT_EQ(a1, a2);
}

TEST_F(LayerGraphImplTest, GenVtxColor_io_input) {

    LayerGraph_Impl to;
    LayerIONames io;

    io.Input("foo", "bar");
    VtxColor a = to.genVtxColor("a", LayerConfig(), io);

    EXPECT_EQ(a, to.genVtxColor("a", LayerConfig(), io));

    EXPECT_NE(a, to.genVtxColor("a ", LayerConfig(), LayerIONames()));

    io.Input("foo", "bar");
    EXPECT_EQ(a, to.genVtxColor("a", LayerConfig(), io));

    io.Input("foo", "x");
    EXPECT_NE(a, to.genVtxColor("a ", LayerConfig(), LayerIONames()));

    io.Input("foo", "bar");
    EXPECT_EQ(a, to.genVtxColor("a", LayerConfig(), io));

    LayerIONames io2;
    io2.Input("x", "bar");
    EXPECT_NE(a, to.genVtxColor("a", LayerConfig(), io2)) << "Match inspite of different keys";
}

TEST_F(LayerGraphImplTest, GenVtxColor_io_output) {

    LayerGraph_Impl to;
    LayerIONames io;

    io.Output("foo", "bar");
    VtxColor a = to.genVtxColor("a", LayerConfig(), io);

    EXPECT_EQ(a, to.genVtxColor("a", LayerConfig(), io));

    EXPECT_NE(a, to.genVtxColor("a ", LayerConfig(), LayerIONames()));

    io.Output("foo", "bar");
    EXPECT_EQ(a, to.genVtxColor("a", LayerConfig(), io));

    io.Output("foo", "x");
    EXPECT_NE(a, to.genVtxColor("a ", LayerConfig(), LayerIONames()));

    io.Output("foo", "bar");
    EXPECT_EQ(a, to.genVtxColor("a", LayerConfig(), io));

    LayerIONames io2;
    io2.Output("x", "bar");
    EXPECT_NE(a, to.genVtxColor("a", LayerConfig(), io2)) << "Match inspite of different keys";
}

TEST_F(LayerGraphImplTest, Sequence_appends) {

    LayerGraph_Impl to;

    std::shared_ptr<base_Layer> a(new LayerA);
    std::shared_ptr<base_Layer> b(new LayerB);

    {
        LayerConfig cfg;
        PTree p;
        p.put("pb", "pb1");
        LayerIONames io;
        io.Input(LayerB::KEY_INPUT_STIMULUS, "outa");
        io.Output(LayerB::KEY_OUTPUT_RESPONSE, "outb");
        to.Add("b", b, cfg, io);
    }
    {
        LayerConfig cfg;
        PTree p;
        p.put("pa", "pa1");
        LayerIONames io;
        io.Input(LayerA::KEY_INPUT_STIMULUS, "ina");
        io.Output(LayerA::KEY_OUTPUT_RESPONSE, "outa");
        to.Add("a", a, cfg, io);
    }
    to.AddOutput("outb");

    std::vector<LayerShared> layers;
    to.Sequence(layers);

    EXPECT_SIZE(2, layers);

    to.Sequence(layers);

    EXPECT_SIZE(2+2, layers) << "sequence not appended as expected.";

    EXPECT_NE(layers[0], layers[1]);
    EXPECT_EQ(layers[0], layers[2]);
    EXPECT_EQ(layers[1], layers[3]);

    to.Sequence(layers);

    EXPECT_SIZE(2+2+2, layers) << "sequence not appended as expected.";

    EXPECT_NE(layers[0], layers[1]);
    EXPECT_EQ(layers[0], layers[2]);
    EXPECT_EQ(layers[1], layers[3]);
    EXPECT_EQ(layers[0+2], layers[2+2]);
    EXPECT_EQ(layers[1+2], layers[3+2]);
}

TEST_F(LayerGraphImplTest, Sequence_linear) {

    LayerGraph_Impl to;

    std::shared_ptr<base_Layer> a(new LayerA);
    std::shared_ptr<base_Layer> b(new LayerB);
    std::shared_ptr<base_Layer> c(new LayerC);
    std::shared_ptr<base_Layer> d(new LayerD);

    {
        LayerConfig cfg;
        PTree p;
        p.put("pb", "pb1");
        LayerIONames io;
        io.Input(LayerB::KEY_INPUT_STIMULUS, "outa");
        io.Output(LayerB::KEY_OUTPUT_RESPONSE, "outb");
        to.Add("b", b, cfg, io);
    }
    {
        LayerConfig cfg;
        PTree p;
        p.put("pc", "pc1");
        LayerIONames io;
        io.Input(LayerC::KEY_INPUT_STIMULUS, "outb");
        io.Output(LayerC::KEY_OUTPUT_RESPONSE, "outc");
        to.Add("c", c, cfg, io);
    }
    {
        LayerConfig cfg;
        PTree p;
        p.put("pa", "pa1");
        LayerIONames io;
        io.Input(LayerA::KEY_INPUT_STIMULUS, "ina");
        io.Output(LayerA::KEY_OUTPUT_RESPONSE, "outa");
        to.Add("a", a, cfg, io);
    }
    {
        LayerConfig cfg;
        LayerIONames io;
        io.Input(LayerD::KEY_INPUT_STIMULUS, "ina");
        io.Output(LayerD::KEY_OUTPUT_RESPONSE, "outd");
        to.Add("d", d, cfg, io);
    }
    to.AddOutput("outc");

    to.Configure();

    std::vector<LayerShared> layers;
    to.Sequence(layers);

    EXPECT_SIZE(3, layers);

    Signal sig;
    sig.Append("ina", Mat1f(1, 1, 1.f));

    EXPECT_FALSE(sig.Exists("outa"));
    EXPECT_FALSE(sig.Exists("outb"));
    EXPECT_FALSE(sig.Exists("outc"));
    EXPECT_FALSE(sig.Exists("outd"));

    layers[0]->Activate(sig);
    layers[0]->Response(sig);

    EXPECT_TRUE(sig.Exists("outa"));
    EXPECT_FALSE(sig.Exists("outb"));
    EXPECT_FALSE(sig.Exists("outc"));
    EXPECT_FALSE(sig.Exists("outd"));

    layers[1]->Activate(sig);
    layers[1]->Response(sig);

    EXPECT_TRUE(sig.Exists("outa"));
    EXPECT_TRUE(sig.Exists("outb"));
    EXPECT_FALSE(sig.Exists("outc"));
    EXPECT_FALSE(sig.Exists("outd"));

    layers[2]->Activate(sig);
    layers[2]->Response(sig);

    EXPECT_TRUE(sig.Exists("outa"));
    EXPECT_TRUE(sig.Exists("outb"));
    EXPECT_TRUE(sig.Exists("outc"));
    EXPECT_FALSE(sig.Exists("outd"));
}

TEST_F(LayerGraphImplTest, Sequence_multiple_paths) {

    LayerGraph_Impl to;

    std::shared_ptr<base_Layer> a(new LayerA);
    std::shared_ptr<base_Layer> b(new LayerB);
    std::shared_ptr<base_Layer> c(new LayerC);
    std::shared_ptr<base_Layer> d(new LayerD);

    {
        LayerConfig cfg;
        PTree p;
        p.put("pb", "pb1");
        LayerIONames io;
        io.Input(LayerB::KEY_INPUT_STIMULUS, "outa");
        io.Output(LayerB::KEY_OUTPUT_RESPONSE, "outb");
        to.Add("b", b, cfg, io);
    }
    {
        LayerConfig cfg;
        PTree p;
        p.put("pc", "pc1");
        LayerIONames io;
        io.Input(LayerC::KEY_INPUT_STIMULUS, "outb");
        io.Output(LayerC::KEY_OUTPUT_RESPONSE, "outc");
        to.Add("c", c, cfg, io);
    }
    {
        LayerConfig cfg;
        PTree p;
        p.put("pa", "pa1");
        LayerIONames io;
        io.Input(LayerA::KEY_INPUT_STIMULUS, "ina");
        io.Output(LayerA::KEY_OUTPUT_RESPONSE, "outa");
        to.Add("a", a, cfg, io);
    }
    {
        LayerConfig cfg;
        LayerIONames io;
        io.Input(LayerD::KEY_INPUT_STIMULUS, "outb");
        io.Output(LayerD::KEY_OUTPUT_RESPONSE, "outd");
        to.Add("d", d, cfg, io);
    }
    to.AddOutput("outc");
    to.AddOutput("outd");

    to.Configure();

    std::vector<LayerShared> layers;
    to.Sequence(layers);

    EXPECT_SIZE(4, layers);

    Signal sig;
    sig.Append("ina", Mat1f(1, 1, 1.f));

    EXPECT_FALSE(sig.Exists("outa"));
    EXPECT_FALSE(sig.Exists("outb"));
    EXPECT_FALSE(sig.Exists("outc"));
    EXPECT_FALSE(sig.Exists("outd"));

    layers[0]->Activate(sig);
    layers[0]->Response(sig);

    EXPECT_TRUE(sig.Exists("outa"));
    EXPECT_FALSE(sig.Exists("outb"));
    EXPECT_FALSE(sig.Exists("outc"));
    EXPECT_FALSE(sig.Exists("outd"));

    layers[1]->Activate(sig);
    layers[1]->Response(sig);

    EXPECT_TRUE(sig.Exists("outa"));
    EXPECT_TRUE(sig.Exists("outb"));
    EXPECT_FALSE(sig.Exists("outc"));
    EXPECT_FALSE(sig.Exists("outd"));

    layers[2]->Activate(sig);
    layers[2]->Response(sig);

    EXPECT_TRUE(sig.Exists("outa"));
    EXPECT_TRUE(sig.Exists("outb"));
    EXPECT_TRUE(sig.Exists("outc") ^ sig.Exists("outd")) << "Expecting only one response";

    layers[3]->Activate(sig);
    layers[3]->Response(sig);

    EXPECT_TRUE(sig.Exists("outa"));
    EXPECT_TRUE(sig.Exists("outb"));
    EXPECT_TRUE(sig.Exists("outc"));
    EXPECT_TRUE(sig.Exists("outd"));
}

TEST_F(LayerGraphImplTest, Sequence_multiple_paths_signal_feature) {

    LayerGraph_Impl to;

    std::shared_ptr<base_Layer> a(new LayerA);
    std::shared_ptr<base_Layer> b(new LayerB);
    std::shared_ptr<base_Layer> c(new LayerC);
    std::shared_ptr<base_Layer> d(new LayerD);

    {
        LayerConfig cfg;
        PTree p;
        p.put("pb", "pb1");
        LayerIONames io;
        io.Input(LayerB::KEY_INPUT_STIMULUS, "outa");
        io.Output(LayerB::KEY_OUTPUT_RESPONSE, "outb");
        to.Add("b", b, cfg, io);
    }
    {
        LayerConfig cfg;
        PTree p;
        p.put("pc", "pc1");
        LayerIONames io;
        io.Input(LayerC::KEY_INPUT_STIMULUS, "outb");
        io.Output(LayerC::KEY_OUTPUT_RESPONSE, "outc");
        to.Add("c", c, cfg, io);
    }
    {
        LayerConfig cfg;
        PTree p;
        p.put("pa", "pa1");
        LayerIONames io;
        io.Input(LayerA::KEY_INPUT_STIMULUS, "ina");
        io.Output(LayerA::KEY_OUTPUT_RESPONSE, "outa");
        to.Add("a", a, cfg, io);
    }
    {
        LayerConfig cfg;
        LayerIONames io;
        io.Input(LayerD::KEY_INPUT_STIMULUS, "outb");
        io.Output(LayerD::KEY_OUTPUT_RESPONSE, "outd");
        to.Add("d", d, cfg, io);
    }
    to.AddOutput("outc");
    to.AddOutput("outd");

    to.Configure();

    std::vector<LayerShared> layers;
    to.Sequence(layers);

    EXPECT_SIZE(4, layers);

    Mat1f in = Mat1f(1, 1, 1.f);
    Mat1f expected = in.clone();

    Signal sig;
    sig.Append("ina", in);

    for(size_t i=0; i<layers.size(); i++) {

        layers[i]->Activate(sig);
        layers[i]->Response(sig);
    }

    expected.push_back(Mat1f(1, 1, static_cast<float>('a')));
    expected.push_back(Mat1f(1, 1, static_cast<float>('b')));

    EXPECT_MAT_EQ(expected.rowRange(0, 2), sig.MostRecentMat1f("outa"));
    EXPECT_MAT_EQ(expected.rowRange(0, 3), sig.MostRecentMat1f("outb"));

    Mat1f expected_c = expected.clone();
    expected_c.push_back(Mat1f(1, 1, static_cast<float>('c')));
    EXPECT_MAT_EQ(expected_c, sig.MostRecentMat1f("outc"));

    Mat1f expected_d = expected.clone();
    expected_d.push_back(Mat1f(1, 1, static_cast<float>('d')));
    EXPECT_MAT_EQ(expected_d, sig.MostRecentMat1f("outd"));
}

TEST_F(LayerGraphImplTest, Reconfigure_empty) {

    EXPECT_EQ(0, LayerGraph_Impl().Reconfigure<int>("x", 5));
}

TEST_F(LayerGraphImplTest, Reconfigure_count) {

    LayerGraph_Impl to;

    std::shared_ptr<base_Layer> a(new LayerA);
    std::shared_ptr<base_Layer> b(new LayerB);
    {
        LayerConfig cfg;
        PTree p;
        p.put("p", 1);
        p.put("pb", "pb1");
        cfg.Params(p);
        LayerIONames io;
        io.Input(LayerB::KEY_INPUT_STIMULUS, "outa");
        io.Output(LayerB::KEY_OUTPUT_RESPONSE, "outb");
        to.Add("b", b, cfg, io);
    }
    {
        LayerConfig cfg;
        PTree p;
        p.put("p", 1);
        p.put("pa", "pa1");
        cfg.Params(p);
        LayerIONames io;
        io.Input(LayerA::KEY_INPUT_STIMULUS, "ina");
        io.Output(LayerA::KEY_OUTPUT_RESPONSE, "outa");
        to.Add("a", a, cfg, io);
    }

    EXPECT_EQ(1, to.Reconfigure("pa", "pa2")) << "Parameter key defined once in one layer only.";
    EXPECT_EQ(2, to.Reconfigure("p", 5)) << "Parameter key defined once for both layers.";
    EXPECT_EQ(0, to.Reconfigure("foo", "bar")) << "No modifications for non-existent paramter key";

    // Reconfigure with new type
    EXPECT_EQ(1, to.Reconfigure("pa", 1));
}

/**
 * @brief inspect signal features after multiple reconfigurations
 */
TEST_F(LayerGraphImplTest, Reconfigure) {

    LayerGraph_Impl to;

    std::shared_ptr<base_Layer> a(new LayerA);
    std::shared_ptr<base_Layer> b(new LayerB);
    {
        LayerConfig cfg;
        PTree p;
        p.put(LayerB::PARAM_CAPITAL, true);
        cfg.Params(p);
        LayerIONames io;
        io.Input(LayerB::KEY_INPUT_STIMULUS, "outa");
        io.Output(LayerB::KEY_OUTPUT_RESPONSE, "outb");
        to.Add("b", b, cfg, io);
    }
    {
        LayerConfig cfg;
        PTree p;
        p.put(LayerA::PARAM_CAPITAL, true);
        p.put(LayerA::PARAM_DUPLICATE, false);
        cfg.Params(p);
        LayerIONames io;
        io.Input(LayerA::KEY_INPUT_STIMULUS, "ina");
        io.Output(LayerA::KEY_OUTPUT_RESPONSE, "outa");
        to.Add("a", a, cfg, io);
    }

    to.AddOutput("outb");
    to.Configure();

    std::vector<LayerShared> layers;
    to.Sequence(layers);

    Mat1f in = Mat1f(1, 1, 1.f);
    Mat1f expected = in.clone();

    Signal sig;
    sig.Append("ina", in);

    for(auto const& l : layers) {

        l->Activate(sig);
        l->Response(sig);
    }

    expected.push_back(Mat1f(VecF({static_cast<float>('A'),
                                   static_cast<float>('B')})).reshape(1, 2));

    // inspect signal features from initial configuration

    EXPECT_MAT_EQ(expected.rowRange(0, 2), sig.MostRecentMat1f("outa"));
    EXPECT_MAT_EQ(expected, sig.MostRecentMat1f("outb"));

    EXPECT_EQ(1, to.Reconfigure(LayerA::PARAM_DUPLICATE, true)) << "Parameter key defined once in one layer only.";

    sig.Clear();
    sig.Append("ina", in);

    for(auto const& l : layers) {

        l->Activate(sig);
        l->Response(sig);
    }

    expected = in.clone();
    expected.push_back(Mat1f(VecF({static_cast<float>('A'),
                                   static_cast<float>('A'),
                                   static_cast<float>('B')})).reshape(1, 3));

    // inspect signal features after altering paramter for 1 layer

    EXPECT_MAT_EQ(expected.rowRange(0, 3), sig.MostRecentMat1f("outa"));
    EXPECT_MAT_EQ(expected, sig.MostRecentMat1f("outb"));

    EXPECT_EQ(2, to.Reconfigure(LayerB::PARAM_CAPITAL, false)) << "Parameter key defined once in one layer only.";

    sig.Clear();
    sig.Append("ina", in);

    for(auto const& l : layers) {

        l->Activate(sig);
        l->Response(sig);
    }

    expected = in.clone();
    expected.push_back(Mat1f(VecF({static_cast<float>('a'),
                                   static_cast<float>('a'),
                                   static_cast<float>('b')})).reshape(1, 3));

    // inspect signal features after altering common paramter

    EXPECT_MAT_EQ(expected.rowRange(0, 3), sig.MostRecentMat1f("outa"));
    EXPECT_MAT_EQ(expected, sig.MostRecentMat1f("outb"));
}

//TEST_F(LayerGraphTest, DISABLED_print) {

//    LayerGraph to;

//    std::shared_ptr<base_Layer> a(new LayerA);
//    std::shared_ptr<base_Layer> b(new LayerB);
//    std::shared_ptr<base_Layer> c(new LayerC);

//    {
//        LayerConfig cfg;
//        PTree p;
//        p.put("pb", "pb1");
//        LayerIONames io;
//        io.Input(LayerA::KEY_INPUT_STIMULUS, "outa");
//        io.Output(LayerA::KEY_INPUT_STIMULUS, "outb");
//        to.Add("b", a, cfg, io);
//    }
//    {
//        LayerConfig cfg;
//        PTree p;
//        p.put("pc", "pc1");
//        LayerIONames io;
//        io.Input(LayerA::KEY_INPUT_STIMULUS, "outb");
//        io.Output(LayerA::KEY_INPUT_STIMULUS, "outc");
//        to.Add("c", a, cfg, io);
//    }
//    {
//        LayerConfig cfg;
//        PTree p;
//        p.put("pa", "pa1");
//        LayerIONames io;
//        io.Input(LayerA::KEY_INPUT_STIMULUS, "ina");
//        io.Output(LayerA::KEY_INPUT_STIMULUS, "outa");
//        to.Add("a", a, cfg, io);
//    }

//    to.print();
//}

} // annonymous namespace for unit tests

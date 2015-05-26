/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/layers/layergraph.h"

#include "gtest/gtest.h"

#include "elm/core/debug_utils.h"
#include "elm/core/exception.h"
#include "elm/core/inputname.h"
#include "elm/core/layerconfig.h"
#include "elm/core/signal.h"
#include "elm/layers/layers_interim/base_featuretransformationlayer.h"
#include "elm/ts/container.h"

using cv::Mat1f;
using namespace elm;

namespace {

class LayerA : public base_FeatureTransformationLayer
{
public:
    void Clear() {}

    void Reconfigure(const LayerConfig &config) {


    }

    void Activate(const Signal &signal) {

        //ELM_COUT_VAR("LayerA::Activate()")
        m_ = Mat1f(1, 1, 3.f);
    }
};

class LayerB : public base_FeatureTransformationLayer
{
public:
    void Clear() {}

    void Reconfigure(const LayerConfig &config) {


    }

    void Activate(const Signal &signal) {

        //ELM_COUT_VAR("LayerB::Activate()")
        m_ = Mat1f(1, 1, 4.f);
    }
};

class LayerC : public base_FeatureTransformationLayer
{
public:
    void Clear() {}

    void Reconfigure(const LayerConfig &config) {


    }

    void Activate(const Signal &signal) {

        //ELM_COUT_VAR("LayerC::Activate()")
        m_ = Mat1f(1, 1, 4.f);
    }
};

class LayerD : public base_FeatureTransformationLayer
{
public:
    void Clear() {}

    void Reconfigure(const LayerConfig &config) {


    }

    void Activate(const Signal &signal) {

        //ELM_COUT_VAR("LayerD::Activate()")
        m_ = Mat1f(1, 1, 5.f);
    }
};

class LayerGraphTest : public ::testing::Test
{

};

TEST_F(LayerGraphTest, ClearActive_empty) {

    EXPECT_NO_THROW(LayerGraph().ClearActive());
}

TEST_F(LayerGraphTest, ClearActive) {

    LayerGraph to;

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

TEST_F(LayerGraphTest, Outputs_empty) {

    EXPECT_TRUE(LayerGraph().Outputs().empty());
}

TEST_F(LayerGraphTest, Outputs_single) {

    LayerGraph to;

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

TEST_F(LayerGraphTest, Inputs_empty) {

    EXPECT_TRUE(LayerGraph().Inputs().empty());
}

TEST_F(LayerGraphTest, Inputs_single) {

    LayerGraph to;

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

TEST_F(LayerGraphTest, Inputs_chained) {

    LayerGraph to;

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
        io.Input(LayerA::KEY_INPUT_STIMULUS, "outa");
        io.Output(LayerA::KEY_OUTPUT_RESPONSE, "outb");
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

TEST_F(LayerGraphTest, Inputs_multiple) {

    LayerGraph to;

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
        io.Input(LayerA::KEY_INPUT_STIMULUS, "inb");
        io.Output(LayerA::KEY_OUTPUT_RESPONSE, "outb");
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
TEST_F(LayerGraphTest, Inputs_single_active_only) {

    LayerGraph to;

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
        io.Input(LayerA::KEY_INPUT_STIMULUS, "inb");
        io.Output(LayerA::KEY_OUTPUT_RESPONSE, "outb");
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

TEST_F(LayerGraphTest, AddOutput) {

    LayerGraph to;

    std::shared_ptr<base_Layer> a(new LayerA);
    std::shared_ptr<base_Layer> b(new LayerB);
    std::shared_ptr<base_Layer> c(new LayerC);
    std::shared_ptr<base_Layer> d(new LayerD);

    {
        LayerConfig cfg;
        PTree p;
        p.put("pb", "pb1");
        LayerIONames io;
        io.Input(LayerA::KEY_INPUT_STIMULUS, "outa");
        io.Output(LayerA::KEY_OUTPUT_RESPONSE, "outb");
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

TEST_F(LayerGraphTest, AddOutput_invalid) {

    LayerGraph to;

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

TEST_F(LayerGraphTest, GenVtxColor_name) {

    LayerGraph to;
    VtxColor a = to.genVtxColor("a", LayerConfig(), LayerIONames());

    EXPECT_EQ(a, to.genVtxColor("a", LayerConfig(), LayerIONames()));
    EXPECT_NE(a, to.genVtxColor("a ", LayerConfig(), LayerIONames()));
    EXPECT_NE(a, to.genVtxColor("a1", LayerConfig(), LayerIONames()));
    EXPECT_NE(a, to.genVtxColor("b", LayerConfig(), LayerIONames()));
}

TEST_F(LayerGraphTest, GenVtxColor_cfg) {

    LayerGraph to;
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

TEST_F(LayerGraphTest, GenVtxColor_io_input) {

    LayerGraph to;
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

TEST_F(LayerGraphTest, GenVtxColor_io_output) {

    LayerGraph to;
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

TEST_F(LayerGraphTest, Sequence_appends) {

    LayerGraph to;

    std::shared_ptr<base_Layer> a(new LayerA);
    std::shared_ptr<base_Layer> b(new LayerB);
    std::shared_ptr<base_Layer> c(new LayerC);

    {
        LayerConfig cfg;
        PTree p;
        p.put("pb", "pb1");
        LayerIONames io;
        io.Input(LayerA::KEY_INPUT_STIMULUS, "outa");
        io.Output(LayerA::KEY_OUTPUT_RESPONSE, "outb");
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

TEST_F(LayerGraphTest, Sequence_linear) {

    LayerGraph to;

    std::shared_ptr<base_Layer> a(new LayerA);
    std::shared_ptr<base_Layer> b(new LayerB);
    std::shared_ptr<base_Layer> c(new LayerC);
    std::shared_ptr<base_Layer> d(new LayerD);

    {
        LayerConfig cfg;
        PTree p;
        p.put("pb", "pb1");
        LayerIONames io;
        io.Input(LayerA::KEY_INPUT_STIMULUS, "outa");
        io.Output(LayerA::KEY_OUTPUT_RESPONSE, "outb");
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


TEST_F(LayerGraphTest, Sequence_multiple_paths) {

    LayerGraph to;

    std::shared_ptr<base_Layer> a(new LayerA);
    std::shared_ptr<base_Layer> b(new LayerB);
    std::shared_ptr<base_Layer> c(new LayerC);
    std::shared_ptr<base_Layer> d(new LayerD);

    {
        LayerConfig cfg;
        PTree p;
        p.put("pb", "pb1");
        LayerIONames io;
        io.Input(LayerA::KEY_INPUT_STIMULUS, "outa");
        io.Output(LayerA::KEY_OUTPUT_RESPONSE, "outb");
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

TEST_F(LayerGraphTest, Reconfigure_empty) {

    EXPECT_EQ(0, LayerGraph().Reconfigure<int>("x", 5));
}

TEST_F(LayerGraphTest, Reconfigure) {

    LayerGraph to;

    std::shared_ptr<base_Layer> a(new LayerA);
    std::shared_ptr<base_Layer> b(new LayerB);
    {
        LayerConfig cfg;
        PTree p;
        p.put("pb", "pb1");
        cfg.Params(p);
        LayerIONames io;
        io.Input(LayerA::KEY_INPUT_STIMULUS, "outa");
        io.Output(LayerA::KEY_OUTPUT_RESPONSE, "outb");
        to.Add("b", b, cfg, io);
    }
    {
        LayerConfig cfg;
        PTree p;
        p.put("pa", "pa1");
        cfg.Params(p);
        LayerIONames io;
        io.Input(LayerA::KEY_INPUT_STIMULUS, "ina");
        io.Output(LayerA::KEY_OUTPUT_RESPONSE, "outa");
        to.Add("a", a, cfg, io);
    }

    to.Reconfigure("pa", "pa2");
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

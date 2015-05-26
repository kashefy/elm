/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/layers/layergraph.h"

#include "gtest/gtest.h"

#include <boost/filesystem.hpp>

#include "elm/core/debug_utils.h"
#include "elm/core/exception.h"
#include "elm/core/inputname.h"
#include "elm/core/layerconfig.h"
#include "elm/core/signal.h"
#include "elm/layers/layers_interim/base_featuretransformationlayer.h"
#include "elm/layers/weightedsum.h"
#include "elm/ts/container.h"
#include "elm/ts/mat_assertions.h"

using cv::Mat1f;
namespace bfs=boost::filesystem;
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

TEST_F(LayerGraphTest, Sequence_appends) {

    LayerGraph to;

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


TEST_F(LayerGraphTest, Sequence_multiple_paths_signal_feature) {

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

    to.ActivateForResponse(layers, sig);

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

TEST_F(LayerGraphTest, Reconfigure_empty) {

    EXPECT_EQ(0, LayerGraph().Reconfigure<int>("x", 5));
}

TEST_F(LayerGraphTest, Reconfigure_count) {

    LayerGraph to;

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

    EXPECT_EQ(1, to.Reconfigure("pa", std::string("pa2"))) << "Parameter key defined once in one layer only.";
    EXPECT_EQ(2, to.Reconfigure("p", 5)) << "Parameter key defined once for both layers.";
    EXPECT_EQ(2, to.Reconfigure("p", 5.1f)) << "Parameter key defined once for both layers.";
    EXPECT_EQ(0, to.Reconfigure("foo", std::string("bar"))) << "No modifications for non-existent paramter key";

    // Reconfigure with new type
    EXPECT_EQ(1, to.Reconfigure("pa", 1));
}

/**
 * @brief inspect signal features after multiple reconfigurations
 */
TEST_F(LayerGraphTest, Reconfigure) {

    LayerGraph to;

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
    to.ActivateForResponse(layers, sig);

    expected.push_back(Mat1f(VecF({static_cast<float>('A'),
                                   static_cast<float>('B')})).reshape(1, 2));

    // inspect signal features from initial configuration

    EXPECT_MAT_EQ(expected.rowRange(0, 2), sig.MostRecentMat1f("outa"));
    EXPECT_MAT_EQ(expected, sig.MostRecentMat1f("outb"));

    EXPECT_EQ(1, to.Reconfigure(LayerA::PARAM_DUPLICATE, true)) << "Parameter key defined once in one layer only.";

    sig.Clear();
    sig.Append("ina", in);
    to.ActivateForResponse(layers, sig);

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
    to.ActivateForResponse(layers, sig);

    expected = in.clone();
    expected.push_back(Mat1f(VecF({static_cast<float>('a'),
                                   static_cast<float>('a'),
                                   static_cast<float>('b')})).reshape(1, 3));

    // inspect signal features after altering common paramter

    EXPECT_MAT_EQ(expected.rowRange(0, 3), sig.MostRecentMat1f("outa"));
    EXPECT_MAT_EQ(expected, sig.MostRecentMat1f("outb"));
}

class LayerGraphSerializationTest : public ::testing::Test
{
protected:
    static void SetUpTestCase() {

        bfs::create_directory("foo");
    }

    static void TearDownTestCase() {

        if(bfs::is_directory("foo")) {

            bfs::remove_all("foo");
        }
    }
};

TEST_F(LayerGraphSerializationTest, Save_invalid) {

    bfs::path p("foo");
    ASSERT_TRUE(bfs::is_directory(p));

    EXPECT_THROW(LayerGraph().Save(p.string()), ExceptionFileIOError);
}

TEST_F(LayerGraphSerializationTest, Load_invalid) {

    bfs::path p("foo");
    ASSERT_TRUE(bfs::is_directory(p));

    EXPECT_THROW(LayerGraph().Load(p.string()), ExceptionFileIOError);

    p = p / "g.txt";

    ASSERT_FALSE(bfs::is_regular_file(p));

    EXPECT_THROW(LayerGraph().Load(p.string()), ExceptionFileIOError);
}

TEST_F(LayerGraphSerializationTest, Serialize) {

    bfs::path p = bfs::path("foo") / "g.txt";

    {
        LayerGraph to;

        std::shared_ptr<base_Layer> a(new WeightedSum);
        std::shared_ptr<base_Layer> b(new WeightedSum);
        {
            LayerConfig cfg;
            PTree p;
            p.put(WeightedSum::PARAM_A, 0.5f);
            p.put(WeightedSum::PARAM_B, 0.f);
            cfg.Params(p);
            LayerIONames io;
            io.Input(WeightedSum::KEY_INPUT_STIMULUS, "outa");
            io.Output(WeightedSum::KEY_OUTPUT_RESPONSE, "outb");
            to.Add("WeightedSum", b, cfg, io);
        }
        {
            LayerConfig cfg;
            PTree p;
            p.put(WeightedSum::PARAM_A, 2.f);
            p.put(WeightedSum::PARAM_B, 1.f);
            cfg.Params(p);
            LayerIONames io;
            io.Input(WeightedSum::KEY_INPUT_STIMULUS, "ina");
            io.Output(WeightedSum::KEY_OUTPUT_RESPONSE, "outa");
            to.Add("WeightedSum", a, cfg, io);
        }

        to.Save(p.string());
    }

    LayerGraph to; // must be a new object
    to.Load(p.string());

    to.AddOutput("outb");
    to.Configure();

    std::vector<LayerShared> layers;
    to.Sequence(layers);

    Signal sig;
    sig.Append("ina", Mat1f(1, 2, 1.f));
    to.ActivateForResponse(layers, sig);

    EXPECT_MAT_EQ(Mat1f(1, 1, 3.f), sig.MostRecentMat1f("outa"));
    EXPECT_MAT_EQ(Mat1f(1, 1, 1.5f), sig.MostRecentMat1f("outb"));
}

} // annonymous namespace for unit tests

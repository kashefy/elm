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
#include "elm/core/inputname.h"
#include "elm/core/layerconfig.h"
#include "elm/layers/layers_interim/base_featuretransformationlayer.h"

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

        ELM_COUT_VAR("LayerA::Activate()")
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

        ELM_COUT_VAR("LayerB::Activate()")
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

        ELM_COUT_VAR("LayerC::Activate()")
        m_ = Mat1f(1, 1, 4.f);
    }
};

class LayerGraphTest : public ::testing::Test
{

};

TEST_F(LayerGraphTest, Test) {

    LayerGraph lg;

    std::shared_ptr<base_Layer> a(new LayerA);
    std::shared_ptr<base_Layer> b(new LayerB);
    std::shared_ptr<base_Layer> c(new LayerC);

    {
        LayerConfig cfg;
        PTree p;
        p.put("pa", "pa1");
        LayerIONames io;
        io.Input(LayerA::KEY_INPUT_STIMULUS, "ina");
        io.Output(LayerA::KEY_INPUT_STIMULUS, "outa");
        lg.Add("a", a, cfg, io);
    }
    {
        LayerConfig cfg;
        PTree p;
        p.put("pb", "pb1");
        LayerIONames io;
        io.Input(LayerA::KEY_INPUT_STIMULUS, "outa");
        io.Output(LayerA::KEY_INPUT_STIMULUS, "outb");
        lg.Add("b", a, cfg, io);
    }
    {
        LayerConfig cfg;
        PTree p;
        p.put("pc", "pc1");
        LayerIONames io;
        io.Input(LayerA::KEY_INPUT_STIMULUS, "outb");
        io.Output(LayerA::KEY_INPUT_STIMULUS, "outc");
        lg.Add("c", a, cfg, io);
    }

    lg.print();
}

} // annonymous namespace for unit tests

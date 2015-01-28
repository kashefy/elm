/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, Elm Library Project
// 3-clause BSD License
//
//M*/
#include "elm/layers/sinkhornbalancing.h"

#include "elm/core/signal.h"
#include "elm/core/layerconfig.h"
#include "elm/layers/layerfactory.h"
#include "elm/ts/layer_assertions.h"

using namespace std;
using namespace cv;
using namespace elm;

namespace {

ELM_INSTANTIATE_LAYER_TYPED_TEST_CASE_P(SinkhornBalancing);

const string NAME_IN_M              = "in";
const string NAME_OUT_M             = "out";
const string NAME_OUT_CONVERGENCE   = "c";

class SinkhornBalancingTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        PTree p;
        p.put(SinkhornBalancing::PARAM_EPSILON, 0.1f);
        p.put(SinkhornBalancing::PARAM_MAX_ITER, 10);

        LayerConfig cfg;
        cfg.Params(p);

        LayerIONames io;
        io.Input(SinkhornBalancing::KEY_INPUT_MAT, NAME_IN_M);
        io.Output(SinkhornBalancing::KEY_OUTPUT_MAT_BALANCED, NAME_OUT_M);
        io.Output(SinkhornBalancing::KEY_OUTPUT_IS_CONVERGED, NAME_OUT_CONVERGENCE);

        to_ = LayerFactory::CreateShared("SinkhornBalancing", cfg, io);

        m_ = Mat1f(3, 4);
        randn(m_, 0.f, 1.f);
        m_ = abs(m_);

        sig_.Append(NAME_IN_M, m_);
    }

    virtual void TearDown()
    {
        sig_.Clear();
    }

    std::shared_ptr<base_Layer> to_; ///< ptr to test object

    Mat1f m_;
    Signal sig_;
};

TEST_F(SinkhornBalancingTest, ActivateAndResponse)
{
    to_->Activate(sig_);
    to_->Response(sig_);



}


} // annonymous namespace for test cases and fixtures

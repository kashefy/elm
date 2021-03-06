/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/layers/sinkhornbalancing.h"

#include "elm/core/signal.h"
#include "elm/core/layerconfig.h"
#include "elm/layers/layerfactory.h"
#include "elm/ts/layer_assertions.h"
#include "elm/ts/mat_assertions.h"

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
        io.Input(SinkhornBalancing::KEY_INPUT_STIMULUS, NAME_IN_M);
        io.Output(SinkhornBalancing::KEY_OUTPUT_MAT_BALANCED, NAME_OUT_M);
        io.Output(SinkhornBalancing::KEY_OUTPUT_IS_CONVERGED, NAME_OUT_CONVERGENCE);

        to_ = LayerFactory::CreateShared("SinkhornBalancing", cfg, io);

        m_ = Mat1f(4, 3);
        randn(m_, 0.f, 1.f);
        m_ = abs(m_);

        sig_.Append(NAME_IN_M, m_);
    }

    virtual void TearDown()
    {
        sig_.Clear();
    }

    LayerShared to_; ///< ptr to test object

    Mat1f m_;
    Signal sig_;
};

TEST_F(SinkhornBalancingTest, Dims)
{
    for(int r=0; r<11; r++) {

        for(int c=0; c<11; c++) {

            m_ = Mat1f::ones(r, c);

            sig_.Append(NAME_IN_M, m_);

            to_->Activate(sig_);
            to_->Response(sig_);

            Mat1f out = sig_.MostRecentMat1f(NAME_OUT_M);

            EXPECT_MAT_DIMS_EQ(out, m_) << "Output's dims should match those of input.";
        }
    }
}

TEST_F(SinkhornBalancingTest, Empty)
{
    sig_.Append(NAME_IN_M, Mat1f());
    to_->Activate(sig_);
    to_->Response(sig_);

    Mat1f out = sig_.MostRecentMat1f(NAME_OUT_M);
    EXPECT_TRUE(out.empty());
    EXPECT_FALSE(static_cast<bool>(sig_.MostRecentMat1f(NAME_OUT_CONVERGENCE)(0)));
}

TEST_F(SinkhornBalancingTest, OutputSum)
{
    for(int r=1; r<11; r++) {

        for(int c=1; c<11; c++) {

            m_ = Mat1f(r, c);
            randn(m_, 0.f, 1.f);
            m_ = abs(m_);

            sig_.Append(NAME_IN_M, m_);

            to_->Activate(sig_);
            to_->Response(sig_);

            Mat1f out = sig_.MostRecentMat1f(NAME_OUT_M);

            EXPECT_FLOAT_EQ(c, sum(out)[0]);
        }
    }
}

TEST_F(SinkhornBalancingTest, ActivateAndResponse)
{
    to_->Activate(sig_);
    to_->Response(sig_);

    Mat1f out = sig_.MostRecentMat1f(NAME_OUT_M);

    EXPECT_FLOAT_EQ(m_.cols, sum(out)[0]) << "output did not some up to expected value.";
}

TEST_F(SinkhornBalancingTest, PreserveReference)
{
    uchar *m_data_ptr = m_.data;
    Mat1f m0 = m_.clone();
    Mat1f m_tl = m_(Rect2i(0, 0, m_.cols-1, m_.rows-1));

    SinkhornBalancing::RowColNormalization(m_tl, 3, 0.0001f);

    EXPECT_EQ(m_data_ptr, m_.data) << "Data now pointing somewhere else.";
    EXPECT_EQ(m_tl.data, m_.data) << "Data now pointing somewhere else.";

    EXPECT_MAT_DIMS_EQ(m_tl.clone(), m_(Rect2i(0, 0, m_.cols-1, m_.rows-1))) << "Unexpected mismatch, verify routine is not changing argument reference.";
    EXPECT_MAT_DIMS_EQ(m_tl.clone(), m0(Rect2i(0, 0, m_.cols-1, m_.rows-1))) << "Unexpected mismatch.";
}


} // annonymous namespace for test cases and fixtures

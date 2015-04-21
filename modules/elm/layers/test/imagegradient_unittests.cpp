/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/layers/imagegradient.h"

#include "gtest/gtest.h"

#include <memory>

#include "elm/core/exception.h"
#include "elm/core/inputname.h"
#include "elm/core/layerconfig.h"
#include "elm/core/signal.h"
#include "elm/ts/layer_assertions.h"
#include "elm/ts/mat_assertions.h"
#include "elm/core/debug_utils.h"

using namespace std;
using namespace cv;
using namespace elm;

namespace {

ELM_INSTANTIATE_LAYER_TYPED_TEST_CASE_P(ImageGradient);

const std::string NAME_IN      = "in";
const std::string NAME_GRAD    = "g";

class ImageGradientTest : public ::testing::Test
{
protected:
    virtual void SetUp() {

        to_.reset(new ImageGradient());

        config_ = LayerConfig();

        // params
        PTree params;
        config_.Params(params);

        // IO
        io_ = LayerIONames();
        io_.Input(ImageGradient::KEY_INPUT_STIMULUS, NAME_IN);
        io_.Output(ImageGradient::KEY_OUTPUT_RESPONSE, NAME_GRAD);

        to_.reset(new ImageGradient(config_));
        to_->IONames(io_);
    }

    shared_ptr<base_Layer> to_; ///< test object
    LayerConfig config_;        ///< default config for tests
    LayerIONames io_;            ///< default I/O for tests
};

TEST_F(ImageGradientTest, Reset_EmptyConfig)
{
    EXPECT_NO_THROW(to_->Reset(LayerConfig())) << "All params are optional, no?";
}

TEST_F(ImageGradientTest, Response_exists)
{
    Mat1f in(10, 10, 1.f);

    Signal sig;
    sig.Append(NAME_IN, in);

    to_->Activate(sig);

    EXPECT_FALSE(sig.Exists(NAME_GRAD));

    to_->Response(sig);

    EXPECT_TRUE(sig.Exists(NAME_GRAD));
}

TEST_F(ImageGradientTest, Response_dims)
{
    const int R=10;
    const int C=10;

    for(int r=2; r<R; r++) {

        for(int c=2; c<C; c++) {

            Mat1f in(r, c, 1.f);

            Signal sig;
            sig.Append(NAME_IN, in);

            to_->Activate(sig);
            to_->Response(sig);

            Mat1f gradient = sig.MostRecentMat1f(NAME_GRAD);

            EXPECT_EQ(r, gradient.rows);
            EXPECT_EQ(c*2, gradient.cols);
        }
    }
}

TEST_F(ImageGradientTest, Invalid_input)
{
    for(int r=0; r<2; r++) {

        for(int c=0; c<2; c++) {

            Mat1f in(r, c, 1.f);

            Signal sig;
            sig.Append(NAME_IN, in);

            EXPECT_THROW(to_->Activate(sig), ExceptionBadDims);
        }
    }
}


} // annonymous namespace for test cases and fixtures


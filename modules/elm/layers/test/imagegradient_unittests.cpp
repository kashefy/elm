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

TEST_F(ImageGradientTest, Gradient)
{
    const int ROWS=3;
    const int COLS=2;

    float data1[ROWS*COLS] = {1.1f,  1.2f,
                              2.3f,  2.2f,
                              -3.1f, 0.f,
                             };
    Mat1f in = Mat1f(ROWS, COLS, data1).clone();

    Signal sig;
    sig.Append(NAME_IN, in);

    to_->Activate(sig);
    to_->Response(sig);

    Mat2f gradient = static_cast<Mat2f>(sig.MostRecentMat1f(NAME_GRAD));

    EXPECT_EQ(ROWS, gradient.rows);
    EXPECT_EQ(COLS, gradient.cols);

    VecMat1f components;
    cv::split(gradient, components);

    Mat1f grad_x = components[0];
    EXPECT_FLOAT_EQ(0.1f, grad_x(0, 0));
    EXPECT_NEAR(-0.1f, grad_x(1, 0), 1e-5);
    EXPECT_FLOAT_EQ(-in(2, 0), grad_x(2, 0));
    EXPECT_MAT_EQ(Mat1f::zeros(ROWS, 1), grad_x.col(COLS-1));

    Mat1f grad_y = components[1];
    EXPECT_FLOAT_EQ(1.2f, grad_y(0, 0));
    EXPECT_FLOAT_EQ(1.0f, grad_y(0, 1));
    EXPECT_FLOAT_EQ(-3.1f-2.3f, grad_y(1, 0));
    EXPECT_NEAR(-2.2f, grad_y(1, 1), 1e-5);
    EXPECT_MAT_EQ(Mat1f::zeros(1, COLS), grad_y.row(ROWS-1));
}

TEST_F(ImageGradientTest, Gradient_nan)
{
    const int ROWS=3;
    const int COLS=2;

    const float NAN_VALUE=std::numeric_limits<float>::quiet_NaN();

    float data1[ROWS*COLS] = {1.1f,  NAN_VALUE,
                              2.3f,  NAN_VALUE,
                              -3.1f, 0.f,
                             };
    Mat1f in = Mat1f(ROWS, COLS, data1).clone();

    Signal sig;
    sig.Append(NAME_IN, in);

    to_->Activate(sig);
    to_->Response(sig);

    Mat2f gradient = static_cast<Mat2f>(sig.MostRecentMat1f(NAME_GRAD));

    EXPECT_EQ(ROWS, gradient.rows);
    EXPECT_EQ(COLS, gradient.cols);

    VecMat1f components;
    cv::split(gradient, components);

    Mat1f grad_x = components[0];

    EXPECT_NE(grad_x(0, 0), grad_x(0, 0));
    EXPECT_NE(grad_x(1, 0), grad_x(1, 0));
    EXPECT_FLOAT_EQ(-in(2, 0), grad_x(2, 0));
    EXPECT_MAT_EQ(Mat1f::zeros(ROWS, 1), grad_x.col(COLS-1));

    Mat1f grad_y = components[1];

    EXPECT_FLOAT_EQ(1.2f, grad_y(0, 0));
    EXPECT_NE(grad_y(0, 1), grad_y(0, 1));
    EXPECT_FLOAT_EQ(-3.1f-2.3f, grad_y(1, 0));
    EXPECT_NE(-grad_y(1, 1), grad_y(1, 1));
    EXPECT_MAT_EQ(Mat1f::zeros(1, COLS), grad_y.row(ROWS-1));
}

} // annonymous namespace for test cases and fixtures


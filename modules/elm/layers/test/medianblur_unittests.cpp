/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/layers/medianblur.h"

#include <memory>

#include "elm/core/exception.h"
#include "elm/core/layerconfig.h"
#include "elm/core/signal.h"
#include "elm/ts/layer_assertions.h"

using namespace std;
using namespace cv;
using namespace elm;

namespace {

ELM_INSTANTIATE_LAYER_TYPED_TEST_CASE_P(MedianBlur);

const string NAME_IN            = "in";
const string NAME_OUT_BLURRED   = "out";

class MedianBlurTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        config_ = LayerConfig();

        // params
        PTree params;
        params.add(MedianBlur::PARAM_APERTURE_SIZE, 5);
        config_.Params(params);

        config_.Input(MedianBlur::KEY_INPUT_STIMULUS, NAME_IN);
        config_.Output(MedianBlur::KEY_OUTPUT_RESPONSE, NAME_OUT_BLURRED);

        to_.reset(new MedianBlur(config_));
    }

    shared_ptr<base_Layer> to_; ///< test object
    LayerConfig config_;        ///< default config for tests
};

TEST_F(MedianBlurTest, Reset_EmptyConfig)
{
    EXPECT_THROW(to_->Reset(LayerConfig()), boost::property_tree::ptree_bad_path);
}

TEST_F(MedianBlurTest, Param_invalid)
{
    int ksize = -7;

    while(ksize++ < 17) {

        PTree params;
        params.add(MedianBlur::PARAM_APERTURE_SIZE, ksize);
        config_.Params(params);

        if(ksize <= 1.f) {

            EXPECT_THROW(to_.reset(new MedianBlur(config_)), ExceptionValueError);
        }
        else if(ksize % 2 != 0) {

            EXPECT_NO_THROW(to_.reset(new MedianBlur(config_)));
        }
        else {

            EXPECT_THROW(to_.reset(new MedianBlur(config_)), ExceptionValueError);
        }
    }
}

TEST_F(MedianBlurTest, Response_exists)
{
    Signal sig;
    sig.Append(NAME_IN, Mat1f(10, 10, 1.f));

    to_->Activate(sig);

    EXPECT_FALSE(sig.Exists(NAME_OUT_BLURRED));

    to_->Response(sig);

    EXPECT_TRUE(sig.Exists(NAME_OUT_BLURRED));
}

TEST_F(MedianBlurTest, Response_dims)
{
    const int R=20;
    const int C=20;

    for(int r=5; r<R; r++) {

        for(int c=5; c<C; c++) {

            Signal sig;
            sig.Append(NAME_IN, Mat1f(r, c, 1.f));

            to_->Activate(sig);
            to_->Response(sig);

            Mat1f blurred = sig.MostRecentMat1f(NAME_OUT_BLURRED);

            EXPECT_EQ(r, blurred.rows);
            EXPECT_EQ(c, blurred.cols);
        }
    }
}

TEST_F(MedianBlurTest, Response_const_valued_input)
{
    for(float v=-3.f; v<=3.f; v+=1.5f) {

        Signal sig;

        Mat1f in(10, 10, v);
        sig.Append(NAME_IN, in);

        to_->Activate(sig);
        to_->Response(sig);

        Mat1f blurred = sig.MostRecentMat1f(NAME_OUT_BLURRED);

        if(v < 0.f) {

            EXPECT_MAT_EQ(Mat1f::zeros(in.rows, in.cols), blurred);
        }
        else {

            EXPECT_MAT_EQ(Mat1f(in.rows, in.cols, round(v)), blurred);
        }
    }
}

/**
 * @brief Inspect values of blurred image
 */
TEST_F(MedianBlurTest, Response_blurred_values)
{
    Signal sig;

    // impulse as stimulus
    Mat1f in(10, 10, 90.f);
    in.colRange(5, 10).setTo(100.f);
    sig.Append(NAME_IN, in);

    to_->Activate(sig);
    to_->Response(sig);

    Mat1f blurred = sig.MostRecentMat1f(NAME_OUT_BLURRED);

    std::cout<<in<<std::endl;
    std::cout<<blurred<<std::endl;
}

} // annonymous namespace for test fixtures and test cases

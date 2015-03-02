/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/layers/medianblur.h"

#include <memory>

#include "elm/core/debug_utils.h"

#include "elm/core/exception.h"
#include "elm/core/layerconfig.h"
#include "elm/core/percentile.h"
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

        EXPECT_MAT_EQ(Mat1f(in.rows, in.cols, v), blurred);
    }
}

TEST_F(MedianBlurTest, Response_const_valued_input_with_nan)
{
    for(float v=-3.f; v<=3.f; v+=1.5f) {

        Signal sig;

        Mat1f in(10, 10, v);

        in(2, 3) = numeric_limits<float>::quiet_NaN();

        sig.Append(NAME_IN, in);

        to_->Activate(sig);
        to_->Response(sig);

        Mat1f blurred = sig.MostRecentMat1f(NAME_OUT_BLURRED);

        EXPECT_MAT_EQ(Mat1f(in.rows, in.cols, v), blurred);
    }
}

/**
 * @brief Inspect values of blurred image, when aperture size
 * is large (e.g. kszie > 5)
 * and we expect a truncation to CV_8U
 */
TEST_F(MedianBlurTest, Response_blurred_values_8u)
{
    const int NB_KSZIE_VALUES = 2;
    int ksize_values[NB_KSZIE_VALUES] = {7, 9};

    for(int i=0; i<NB_KSZIE_VALUES; i++) {

        int ksize = ksize_values[i];
        ASSERT_GT(ksize, 5); // sanity check

        PTree params;
        params.add(MedianBlur::PARAM_APERTURE_SIZE, ksize);
        config_.Params(params);
        to_.reset(new MedianBlur(config_));

        Mat1b tmp(ksize, ksize);
        randn(tmp, 125.f, 125.f);
        Mat1f in = tmp;

        Signal sig;
        sig.Append(NAME_IN, in);

        to_->Activate(sig);
        to_->Response(sig);

        Mat1f blurred = sig.MostRecentMat1f(NAME_OUT_BLURRED);

        float median = Percentile().CalcPercentile(in.reshape(1, 1), 0.5f);

        EXPECT_FLOAT_EQ(median, blurred(ksize/2, ksize/2))
                << "Unexpected value at the centre of blurred image with ksize=" << ksize;
    }
}

/**
 * @brief Inspect values of blurred image when input matches apertrue size
 * Ignore off-center elements
 */
TEST_F(MedianBlurTest, Response_blurred_values_median_center)
{
    const int NB_KSZIE_VALUES = 2;
    int ksize_values[NB_KSZIE_VALUES] = {3, 5};

    for(int i=0; i<NB_KSZIE_VALUES; i++) {

        int ksize = ksize_values[i];

        PTree params;
        params.add(MedianBlur::PARAM_APERTURE_SIZE, ksize);
        config_.Params(params);
        to_.reset(new MedianBlur(config_));

        Mat1f in(ksize, ksize);
        randn(in, 0.f, 100.f);

        Signal sig;
        sig.Append(NAME_IN, in);

        to_->Activate(sig);
        to_->Response(sig);

        Mat1f blurred = sig.MostRecentMat1f(NAME_OUT_BLURRED);

        float median = Percentile().CalcPercentile(in.reshape(1, 1), 0.5f);

        EXPECT_FLOAT_EQ(median, blurred(ksize/2, ksize/2))
                << "Unexpected value at the centre of blurred image with ksize=" << ksize;
    }
}

TEST_F(MedianBlurTest, Response_blurred_values_median_center_with_nan)
{
    const int NB_KSZIE_VALUES = 2;
    int ksize_values[NB_KSZIE_VALUES] = {3, 5};

    for(int i=0; i<NB_KSZIE_VALUES; i++) {

        int ksize = ksize_values[i];

        PTree params;
        params.add(MedianBlur::PARAM_APERTURE_SIZE, ksize);
        config_.Params(params);
        to_.reset(new MedianBlur(config_));

        Mat1f in(ksize, ksize);
        randn(in, 0.f, 100.f);
        in(ksize/2-1, ksize/2-1) = numeric_limits<float>::quiet_NaN();

        Signal sig;
        sig.Append(NAME_IN, in);

        to_->Activate(sig);
        to_->Response(sig);

        Mat1f blurred = sig.MostRecentMat1f(NAME_OUT_BLURRED);

//        ELM_COUT_VAR(in);
//        ELM_COUT_VAR(blurred);

        EXPECT_EQ(1, countNonZero(isnan(blurred)));
        EXPECT_EQ(uchar(255), isnan(blurred)(ksize/2-1, ksize/2-1));

        VecF non_nan_values;
        Mat1b mask_not_nan = is_not_nan(in);
        for(size_t j=0; j<in.total(); j++) {

            if(mask_not_nan(j)) {

                non_nan_values.push_back(in(j));
            }
        }

        //ELM_COUT_VAR(Mat1f(non_nan_values).reshape(1, 1));
        float median_no_nan = Percentile().CalcPercentile(Mat1f(non_nan_values).reshape(1, 1), 0.5f);
        float median_with_nan = Percentile().CalcPercentile(in.reshape(1, 1), 0.5f);

        bool is_match = (blurred(ksize/2, ksize/2) == median_no_nan) ||
                (blurred(ksize/2, ksize/2) == median_with_nan) ||
                (blurred(ksize/2, ksize/2-1) == median_no_nan) ||
                (blurred(ksize/2, ksize/2-1) == median_with_nan)
                ;
        EXPECT_TRUE(is_match)
                << "Could not find median value in rough centre of blurred image with ksize=" << ksize;
    }
}

} // annonymous namespace for test fixtures and test cases

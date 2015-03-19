/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/layers/mlp.h"

#include <memory>

#include "elm/core/debug_utils.h"

#include "elm/core/boost/translators/transl_str_cvtermcriteria.h"
#include "elm/core/boost/translators/transl_str_veci.h"
#include "elm/core/cv/mat_utils_inl.h"
#include "elm/core/exception.h"
#include "elm/core/layerconfig.h"
#include "elm/core/percentile.h"
#include "elm/core/signal.h"
#include "elm/ts/layer_assertions.h"

using namespace std;
using namespace cv;
using namespace elm;

namespace {

ELM_INSTANTIATE_LAYER_TYPED_TEST_CASE_P(MLP);

const string NAME_IN             = "in";
const string NAME_OUT_PREDICTION = "out";

class MLPTrainTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        config_ = LayerConfig();

        // params
        {
            PTree params;

            VecI arch = {2, 2, 1};

            params.add(MLP::PARAM_ARCH, arch);
            params.add(MLP::PARAM_BP_DW_SCALE, 0.05f);
            params.add(MLP::PARAM_BP_MOMENT_SCALE, 0.05f);

            CvTermCriteria criteria;
            criteria.max_iter = 1000;
            criteria.epsilon = 0.00001f;
            criteria.type = CV_TERMCRIT_ITER | CV_TERMCRIT_EPS;

            params.add(MLP::PARAM_TERM_CRITERIA, criteria);

            config_.Params(params);
        }

        config_.Input(MLP::KEY_INPUT_STIMULUS, NAME_IN);
        config_.Output(MLP::KEY_OUTPUT_RESPONSE, NAME_OUT_PREDICTION);

        to_.reset(new MLP(config_));
    }

    shared_ptr<base_Layer> to_; ///< test object
    LayerConfig config_;        ///< default config for tests
};

TEST_F(MLPTrainTest, Reset_EmptyConfig)
{
    EXPECT_THROW(to_->Reset(LayerConfig()), boost::property_tree::ptree_bad_path);
}

TEST_F(MLPTrainTest, Param_invalid_layers_too_few)
{
    PTree params = config_.Params();
    {
        VecI arch = {2};
        params.put(MLP::PARAM_ARCH, arch);
        config_.Params(params);
        EXPECT_THROW(to_.reset(new MLP(config_)), ExceptionBadDims);
    }
    {
        VecI arch = {200};
        params.put(MLP::PARAM_ARCH, arch);
        config_.Params(params);
        EXPECT_THROW(to_.reset(new MLP(config_)), ExceptionBadDims);
    }
}

TEST_F(MLPTrainTest, Param_invalid_layers_none)
{
    PTree params = config_.Params();
    {
        params.put(MLP::PARAM_ARCH, VecI());
        config_.Params(params);
        EXPECT_THROW(to_.reset(new MLP(config_)), ExceptionBadDims);
    }
}

//TEST_F(MLPTest, Response_exists)
//{
//    Signal sig;
//    sig.Append(NAME_IN, Mat1f(10, 10, 1.f));

//    to_->Activate(sig);

//    EXPECT_FALSE(sig.Exists(NAME_OUT_BLURRED));

//    to_->Response(sig);

//    EXPECT_TRUE(sig.Exists(NAME_OUT_BLURRED));
//}

//TEST_F(MLPTest, Response_dims)
//{
//    const int R=20;
//    const int C=20;

//    for(int r=5; r<R; r++) {

//        for(int c=5; c<C; c++) {

//            Signal sig;
//            sig.Append(NAME_IN, Mat1f(r, c, 1.f));

//            to_->Activate(sig);
//            to_->Response(sig);

//            Mat1f blurred = sig.MostRecentMat1f(NAME_OUT_BLURRED);

//            EXPECT_EQ(r, blurred.rows);
//            EXPECT_EQ(c, blurred.cols);
//        }
//    }
//}

//TEST_F(MLPTest, Response_const_valued_input)
//{
//    for(float v=-3.f; v<=3.f; v+=1.5f) {

//        Signal sig;

//        Mat1f in(10, 10, v);
//        sig.Append(NAME_IN, in);

//        to_->Activate(sig);
//        to_->Response(sig);

//        Mat1f blurred = sig.MostRecentMat1f(NAME_OUT_BLURRED);

//        EXPECT_MAT_EQ(Mat1f(in.rows, in.cols, v), blurred);
//    }
//}

//TEST_F(MedianBlurTest, Response_const_valued_input_with_nan)
//{
//    for(float v=-3.f; v<=3.f; v+=1.5f) {

//        Signal sig;

//        Mat1f in(10, 10, v);

//        in(2, 3) = numeric_limits<float>::quiet_NaN();

//        sig.Append(NAME_IN, in);

//        to_->Activate(sig);
//        to_->Response(sig);

//        Mat1f blurred = sig.MostRecentMat1f(NAME_OUT_BLURRED);

//        EXPECT_EQ(1, countNonZero(elm::isnan(blurred)));
//        EXPECT_EQ(uchar(255), elm::isnan(blurred)(2, 3));

//        in(2, 3) = blurred(2, 3) = v;
//        EXPECT_MAT_EQ(in, blurred);
//    }
//}

} // annonymous namespace for test fixtures and test cases

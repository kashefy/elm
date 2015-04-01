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

class MLPInitTest : public ::testing::Test
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

        io_ = LayerIONames();
        io_.Input(MLP::KEY_INPUT_STIMULUS, NAME_IN);
        io_.Output(MLP::KEY_OUTPUT_RESPONSE, NAME_OUT_PREDICTION);

        to_.reset(new MLP(config_));
        to_->IONames(io_);
    }

    shared_ptr<base_Layer> to_; ///< test object
    LayerConfig config_;        ///< default config for tests
    LayerIONames io_;
};

TEST_F(MLPInitTest, Reset_EmptyConfig)
{
    EXPECT_THROW(to_->Reset(LayerConfig()), boost::property_tree::ptree_bad_path);
}

TEST_F(MLPInitTest, Param_invalid_layers_too_few)
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

TEST_F(MLPInitTest, Param_invalid_layers_none)
{
    PTree params = config_.Params();
    {
        params.put(MLP::PARAM_ARCH, VecI());
        config_.Params(params);
        EXPECT_THROW(to_.reset(new MLP(config_)), ExceptionBadDims);
    }
}

class MLPTrainTest : public MLPInitTest
{
public:
    void GenerateSeparableBinary(int n, int feat_dims, cv::Mat1f &feat, cv::Mat1f &labels)
    {
        const float NEG_CLASS = -1.f;
        const float POS_CLASS = 1.f;

        feat = Mat1f(n, feat_dims);
        labels = Mat1f(n, 1);

        Mat1f feat_neg(n/2, feat_dims);
        cv::randn(feat_neg, -1.f, 1.f);
        Mat1f labels_neg(feat_neg.rows, 1, NEG_CLASS);

        Mat1f feat_pos(n/2, feat_dims);
        cv::randn(feat_pos, 1.f, 1.f);
        Mat1f labels_pos(feat_pos.rows, 1, POS_CLASS);

        Mat1f feat_concat, labels_concat;

        vconcat(feat_neg, feat_pos, feat_concat);
        vconcat(labels_neg, labels_pos, labels_concat);

        Mat1i idx = ARange_<int>(0, feat_concat.rows, 1);
        cv::randShuffle(idx);

        for (int src_idx=0; src_idx<n; src_idx++) {

            int dst_idx = idx(src_idx);
            feat_concat.row(src_idx).copyTo(feat.row(dst_idx));
            labels_concat.row(src_idx).copyTo(labels.row(dst_idx));
        }
    }

    void GenerateSeparableMultiClass(int nb_classes, int n, int feat_dims, cv::Mat1f &feat, cv::Mat1f &labels)
    {
        const float NEG_CLASS = -1.f;
        const float POS_CLASS = 1.f;

        feat = Mat1f(n, feat_dims);
        labels = Mat1f(n, 1);

        float mu_start = -2.f;
        float mu_incr = 1.f;

        Mat1f feat_concat, labels_concat;

        for (int i=0; i<; i++) {

            Mat1f feati(n/, feat_dims);
            cv::randn(feati, -2.f, 1.f);
            Mat1f labelsi(feati.rows, 1, 0.f);
            labelsi.col(0).setTo(1.f);

            if(i > 0) {

                vconcat(feat_concat, feati, feat_concat);
                vconcat(labels_concat, labelsi, labels_concat);
            }
            else {

                feat_concat = feati;
                labels_concat = labelsi;
            }
        }

        Mat1i idx = ARange_<int>(0, feat_concat.rows, 1);
        cv::randShuffle(idx);

        for (int src_idx=0; src_idx<n; src_idx++) {

            int dst_idx = idx(src_idx);
            feat_concat.row(src_idx).copyTo(feat.row(dst_idx));
            labels_concat.row(src_idx).copyTo(labels.row(dst_idx));
        }
    }

protected:
    virtual void SetUp()
    {
        MLPInitTest::SetUp();

        // params
        {
            PTree params = config_.Params();

            VecI arch = {2, 2, 1};
            params.put(MLP::PARAM_ARCH, arch);

            config_.Params(params);
        }

        to_.reset(new MLP(config_));
        to_->IONames(io_);
    }
};

TEST_F(MLPTrainTest, Train_separable_binary)
{
    const int NB_TRAIN=100;
    const int FEAT_DIMS=2;

    Mat1f train_feat, train_labels;
    GenerateSeparableBinary(NB_TRAIN, FEAT_DIMS,
                            train_feat, train_labels);

    ELM_DYN_CAST(base_LearningLayer, to_)->Learn(train_feat, train_labels);

    const int NB_TEST=100;
    Mat1f test_feat, test_labels;
    GenerateSeparableBinary(NB_TEST, FEAT_DIMS,
                            test_feat, test_labels);

    Mat1i confusion = Mat1i::zeros(2, 2);
    const int TP = 0;
    const int FN = 1;
    const int FP = 2;
    const int TN = 3;

    Mat1f result(NB_TEST, 2);

    for (int i=0; i<NB_TEST; i++) {

        Mat1f test_feat_sample = test_feat.row(i);

        float target_label = test_labels.row(i)(0);

        Signal sig;
        sig.Append(NAME_IN, test_feat_sample);

        EXPECT_FALSE(sig.Exists(NAME_OUT_PREDICTION));

        to_->Activate(sig);
        to_->Response(sig);

        EXPECT_TRUE(sig.Exists(NAME_OUT_PREDICTION));

        Mat1f response = sig.MostRecentMat1f(NAME_OUT_PREDICTION);

        EXPECT_MAT_DIMS_EQ(response, Size2i(1, 1));

        float predicted = response(0);

        result(i, 0) = target_label;
        result(i, 1) = predicted;

        if(target_label >= 0.f && predicted >= 0.f) {

            confusion(TP)++;
        }
        else if(target_label < 0.f && predicted < 0.f) {

            confusion(TN)++;
        }
        else {

            confusion(target_label < 0.f? FP : FN)++;
        }
    }

    //ELM_COUT_VAR(confusion);

    ASSERT_EQ(NB_TEST/2, cv::sum(confusion.row(0))[0]);
    ASSERT_EQ(NB_TEST/2, cv::sum(confusion.row(1))[0]);

    EXPECT_GT(confusion(TP), confusion(FP));
    EXPECT_GT(confusion(TN), confusion(FN));
}

TEST_F(MLPTrainTest, Train_separable_multiclass)
{
    const int NB_CLASSES=4;
    const int NB_TRAIN=120;
    const int FEAT_DIMS=2;

    Mat1f train_feat, train_labels;
    GenerateSeparableMultiClass(NB_CLASSES, NB_TRAIN, FEAT_DIMS,
                                train_feat, train_labels);

    ELM_DYN_CAST(base_LearningLayer, to_)->Learn(train_feat, train_labels);

    const int NB_TEST=120;
    Mat1f test_feat, test_labels;
    GenerateSeparableBinary(NB_CLASSES, NB_TEST, FEAT_DIMS,
                            test_feat, test_labels);

//    for(int c=0; c<NB_CLASSES; c++) {

//        Mat1i confusion = Mat1i::zeros(2, 2);
//        const int TP = 0;
//        const int FN = 1;
//        const int FP = 2;
//        const int TN = 3;

//        Mat1f result(NB_TEST, 2);

//        for (int i=0; i<NB_TEST; i++) {

//            Mat1f test_feat_sample = test_feat.row(i);

//            float target_label = test_labels.row(i)(0);

//            Signal sig;
//            sig.Append(NAME_IN, test_feat_sample);

//            EXPECT_FALSE(sig.Exists(NAME_OUT_PREDICTION));

//            to_->Activate(sig);
//            to_->Response(sig);

//            EXPECT_TRUE(sig.Exists(NAME_OUT_PREDICTION));

//            Mat1f response = sig.MostRecentMat1f(NAME_OUT_PREDICTION);

//            EXPECT_MAT_DIMS_EQ(response, Size2i(1, 1));

//            float predicted = response(0);

//            result(i, 0) = target_label;
//            result(i, 1) = predicted;

//            if(target_label >= 0.f && predicted >= 0.f) {

//                confusion(TP)++;
//            }
//            else if(target_label < 0.f && predicted < 0.f) {

//                confusion(TN)++;
//            }
//            else {

//                confusion(target_label < 0.f? FP : FN)++;
//            }
//        }

//        //ELM_COUT_VAR(confusion);

//        ASSERT_EQ(NB_TEST/2, cv::sum(confusion.row(0))[0]);
//        ASSERT_EQ(NB_TEST/2, cv::sum(confusion.row(1))[0]);

//        EXPECT_GT(confusion(TP), confusion(FP));
//        EXPECT_GT(confusion(TN), confusion(FN));
//    }
}

} // annonymous namespace for test fixtures and test cases

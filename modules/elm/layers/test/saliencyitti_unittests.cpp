/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/layers/saliencyitti.h"

#include <opencv2/highgui/highgui.hpp>

#include "elm/core/layerconfig.h"
#include "elm/core/cv/mat_utils.h"
#include "elm/core/signal.h"
#include "elm/io/synth.h"
#include "elm/io/readmnist.h"
#include "elm/ts/layer_assertions.h"
#include "elm/ts/mat_assertions.h"

using namespace cv;
using namespace std;
using namespace elm;

namespace {

ELM_INSTANTIATE_LAYER_TYPED_TEST_CASE_P(SaliencyItti);

/**
 * @brief class for drawing L shapes to use as simple test data
 */
class SynthL : public base_Synth
{
public:
    SynthL()
        : rows_(-1),
          cols_(-1)
    {}

    /**
     * @brief Set parameters
     * @param rows no. of rows
     * @param cols no. of columns
     */
    void Reset(int rows, int cols)
    {
        rows_ = rows;
        cols_ = cols;
        bars_.Reset(rows_, cols_, 2);
    }

    /**
     * @brief Next image
     * @param[out] L'shaped image
     */
    virtual void Next(cv::Mat &feature, cv::Mat &label)
    {
        Mat horizontal, vertical;
        bars_.Draw(0, horizontal);
        bars_.Draw(90, vertical);

        Mat1b tmp = Mat1b::zeros(rows_, cols_/2-3);
        tmp.copyTo(horizontal.colRange(cols_-tmp.cols, cols_));

        tmp = Mat1b::zeros(rows_/2-3, cols_);
        tmp.copyTo(vertical.rowRange(rows_-tmp.rows, rows_));

        feature = horizontal + vertical;
    }

protected:
    int rows_;      ///< no. of rows in bar image
    int cols_;      ///< no. of columns in bar image
    SynthBars bars_;
};

class SaliencyIttiTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        // Setup layer
        cfg_ = LayerConfig();

        // params
        PTree params;
        cfg_.Params(params);

        // IO
        cfg_.Input(SaliencyItti::KEY_INPUT_STIMULUS, NAME_SCENE_);
        cfg_.Output(SaliencyItti::KEY_OUTPUT_SALIENCY, NAME_SALIENCY_);
        cfg_.Output(SaliencyItti::KEY_OUTPUT_SALIENT_LOC, NAME_LOC_);

        to_.Reset(cfg_);
        to_.IONames(cfg_);
    }

    SaliencyItti to_;   ///< test object
    LayerConfig cfg_;

    static const string NAME_SCENE_;    ///< name for scene stimulus in signal
    static const string NAME_SALIENCY_; ///< name for saliency map in signal
    static const string NAME_LOC_;      ///< name for salient location in signal
};
const string SaliencyIttiTest::NAME_SCENE_      = "in";
const string SaliencyIttiTest::NAME_SALIENCY_   = "saliency";
const string SaliencyIttiTest::NAME_LOC_        = "loc";

TEST_F(SaliencyIttiTest, Reconfigure)
{
    EXPECT_NO_THROW(to_.Reconfigure(cfg_));
}

TEST_F(SaliencyIttiTest, Activate)
{
    const int NB_SAMPLES = 1e3;
    const int WIDTH = 56;
    const int HEIGHT = 56;
    SynthL l;
    l.Reset(HEIGHT, WIDTH);
    Mat1f sampled_saliency(HEIGHT, WIDTH, 0.f);
    Signal signal;
    Mat img, scene, tmp;
    l.Next(img, tmp);
    img.convertTo(scene, CV_32FC1, 1./255.);

    signal.Append(NAME_SCENE_, static_cast<Mat1f>(scene));
    to_.Activate(signal);

    for(int i=0; i<NB_SAMPLES; i++) {

        to_.Response(signal);
        Mat1i loc_mat = signal.MostRecentMat1f(NAME_LOC_);
        sampled_saliency(loc_mat(1), loc_mat(0))++;
    }

//    imshow("sal", elm::ConvertTo8U(signal.MostRecent(NAME_SALIENCY_)));
//    imshow("sam", elm::ConvertTo8U(sampled_saliency));
//    waitKey();
}

TEST_F(SaliencyIttiTest, Activate_scene_zeros)
{
    const int NB_SAMPLES = 2e4;
    const int WIDTH = 56;
    const int HEIGHT = 56;

    Mat1f sampled_saliency(HEIGHT, WIDTH, 0.f);
    Signal signal;
    Mat1f scene = Mat1f::zeros(HEIGHT, WIDTH);

    signal.Append(NAME_SCENE_, scene);
    to_.Activate(signal);

    for(int i=0; i<NB_SAMPLES; i++) {

        to_.Response(signal);
        Mat1i loc_mat = signal.MostRecentMat1f(NAME_LOC_);
        sampled_saliency(loc_mat(1), loc_mat(0))++;
    }

    sampled_saliency /= NB_SAMPLES;

    Mat m, s;
    cv::meanStdDev(sampled_saliency, m, s);

    EXPECT_GT(m.at<double>(0), 0.);
    EXPECT_NEAR(0., m.at<double>(0), 0.01);
}

TEST_F(SaliencyIttiTest, Response_dims)
{
    for(int r=9; r<14; r++) {

        for(int c=9; c<14; c++) {

            Signal signal;
            Mat1f scene = Mat1f::zeros(r, c);

            signal.Append(NAME_SCENE_, scene);
            to_.Activate(signal);
            to_.Response(signal);

            EXPECT_MAT_DIMS_EQ(signal.MostRecentMat1f(NAME_SALIENCY_), Size2i(c, r));
            EXPECT_MAT_DIMS_EQ(signal.MostRecentMat1f(NAME_LOC_), Size2i(2, 1));
        }
    }
}

TEST_F(SaliencyIttiTest, DISABLED_DisplayApplyMNIST)
{
    ReadMNISTImages r;
    r.ReadHeader("/media/206CDC456CDC177E/Users/woodstock/dev/data/MNIST/t10k-images.idx3-ubyte");
    Signal signal;

    Mat img, scene;
    img = r.Next();
    imshow("img", img);
    img.convertTo(scene, CV_32FC1, 1./255.);
    signal.Append(NAME_SCENE_, static_cast<Mat1f>(scene));

    to_.Activate(signal);
    to_.Response(signal);

    imshow("sal", elm::ConvertTo8U(signal.MostRecentMat1f(NAME_SALIENCY_)));

    Mat1i loc_mat = signal.MostRecentMat1f(NAME_LOC_);
    cout<<loc_mat<<endl;

    waitKey();
}

} // annonymous namespace for test cases and fixtures

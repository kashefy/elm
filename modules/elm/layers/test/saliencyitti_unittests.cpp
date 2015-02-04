/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/layers/saliencyitti.h"

#include <opencv2/highgui.hpp>

#include "elm/core/layerconfig.h"
#include "elm/core/cv/mat_utils.h"
#include "elm/core/signal.h"
#include "elm/io/synth.h"
#include "elm/io/readmnist.h"
#include "elm/ts/layer_assertions.h"

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
    SynthL() {}

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

        LayerConfig config;
        // params
        PTree params;
        config.Params(params);

        // IO
        config.Input(SaliencyItti::KEY_INPUT_STIMULUS, NAME_SCENE_);
        config.Output(SaliencyItti::KEY_OUTPUT_SALIENCY, NAME_SALIENCY_);
        config.Output(SaliencyItti::KEY_OUTPUT_SALIENT_LOC, NAME_LOC_);

        to_.Reset(config);
        to_.IONames(config);
    }

    SaliencyItti to_;   ///< test object

    static const string NAME_SCENE_;    ///< name for scene stimulus in signal
    static const string NAME_SALIENCY_; ///< name for saliency map in signal
    static const string NAME_LOC_;      ///< name for salient location in signal
};
const string SaliencyIttiTest::NAME_SCENE_      = "in";
const string SaliencyIttiTest::NAME_SALIENCY_   = "saliency";
const string SaliencyIttiTest::NAME_LOC_        = "loc";

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
        Mat1i loc_mat = signal.MostRecentMat(NAME_LOC_);
        sampled_saliency(loc_mat(1), loc_mat(0))++;
    }

//    imshow("sal", elm::ConvertTo8U(signal.MostRecent(NAME_SALIENCY_)));
//    imshow("sam", elm::ConvertTo8U(sampled_saliency));
//    waitKey();
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

    imshow("sal", elm::ConvertTo8U(signal.MostRecentMat(NAME_SALIENCY_)));

    Mat1i loc_mat = signal.MostRecentMat(NAME_LOC_);
    cout<<loc_mat<<endl;

    waitKey();
}

} // annonymous namespace for test cases and fixtures

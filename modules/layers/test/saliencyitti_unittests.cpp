#include "layers/saliencyitti.h"

#include "core/signal.h"
#include "io/synth.h"
#include "io/readmnist.h"
#include "ts/ts.h"

using namespace cv;
using namespace std;

class SaliencyIttiTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        // Setup layer
        to_.Reset();

        LayerConfig config;
        // params
        PTree params;
        config.Params(params);

        // IO
        config.Input(SaliencyItti::KEY_INPUT_SCENE, NAME_SCENE_);
        config.Output(SaliencyItti::KEY_OUTPUT_SALIENCY, NAME_SALIENCY_);
        config.Output(SaliencyItti::KEY_OUTPUT_SALIENT_LOC, NAME_LOC_);

        to_.Reconfigure(config);
    }

    SaliencyItti to_;   ///< test object

    static const string NAME_SCENE_;    ///< name for scene stimulus in signal
    static const string NAME_SALIENCY_; ///< name for saliency map in signal
    static const string NAME_LOC_;      ///< name for salient location in signal
};
const string SaliencyIttiTest::NAME_SCENE_      = "in";
const string SaliencyIttiTest::NAME_SALIENCY_   = "saliency";
const string SaliencyIttiTest::NAME_LOC_        = "loc";

TEST_F(SaliencyIttiTest, DISABLED_Apply)
{
    SynthBars bars;
    bars.Reset(28, 28, 1);
    Signal signal;


    float angle = 0.f;
    int expected_index = 0;
    //while(angle < 180.f) {

        Mat img, scene;
        bars.Draw(angle, img);
        img.convertTo(scene, CV_32FC1, 1./255.);
        signal.Append(NAME_SCENE_, scene);

        to_.Stimulus(signal);
        to_.Apply();
        to_.Response(signal);

//        const int N=10;
//        Mat1f counts = Mat1f::zeros(1, static_cast<int>(in_.total())*NB_KERNELS_);
//        for(int i=0; i<N; i++) {

//            Mat pop_code = to_.PopCode();
//            EXPECT_MAT_DIMS_EQ(pop_code, counts);
//            EXPECT_LE(cv::sum(pop_code)(0), in_.total())
//                    << "Encountered oversampling for same input element.";
//            cv::add(pop_code, counts, counts);
//        }

//        counts = counts.reshape(1, in_.total());

//        // for which did the bar respond the most?
//        cv::Mat1i col_sums(1, NB_KERNELS_);
//        for(int i=0; i<counts.cols; i++) {

//            col_sums(i) = static_cast<int>(cv::sum(counts.col(i))(0));
//        }

//        double min_val;
//        int min_idx[2] = {-1, -1};
//        int max_idx[2] = {-1, -1};
//        cv::minMaxIdx(col_sums, &min_val, 0, min_idx, max_idx);

//        EXPECT_EQ(expected_index, max_idx[1]);
//        EXPECT_NE(min_idx[1], max_idx[1]);

        angle += 90.;
        expected_index++;
    //}
}

#include <opencv2/highgui.hpp>
#include "core/mat_utils.h"
TEST_F(SaliencyIttiTest, ApplyMNIST)
{
    ReadMNISTImages r;
    r.ReadHeader("/media/206CDC456CDC177E/Users/woodstock/dev/data/MNIST/t10k-images.idx3-ubyte");
    Signal signal;

    float angle = 0.f;
    int expected_index = 0;
    //while(angle < 180.f) {

        Mat img, scene;
        img = r.Next();
        imshow("img", img);
        img.convertTo(scene, CV_32FC1, 1./255.);
        signal.Append(NAME_SCENE_, scene);

        to_.Stimulus(signal);
        to_.Apply();
        to_.Response(signal);

        imshow("sal", sem::ConvertTo8U(signal.MostRecent(NAME_SALIENCY_)));

        Mat1i loc_mat = signal.MostRecent(NAME_LOC_);
        cout<<loc_mat<<endl;

        cv::waitKey();

//        const int N=10;
//        Mat1f counts = Mat1f::zeros(1, static_cast<int>(in_.total())*NB_KERNELS_);
//        for(int i=0; i<N; i++) {

//            Mat pop_code = to_.PopCode();
//            EXPECT_MAT_DIMS_EQ(pop_code, counts);
//            EXPECT_LE(cv::sum(pop_code)(0), in_.total())
//                    << "Encountered oversampling for same input element.";
//            cv::add(pop_code, counts, counts);
//        }

//        counts = counts.reshape(1, in_.total());

//        // for which did the bar respond the most?
//        cv::Mat1i col_sums(1, NB_KERNELS_);
//        for(int i=0; i<counts.cols; i++) {

//            col_sums(i) = static_cast<int>(cv::sum(counts.col(i))(0));
//        }

//        double min_val;
//        int min_idx[2] = {-1, -1};
//        int max_idx[2] = {-1, -1};
//        cv::minMaxIdx(col_sums, &min_val, 0, min_idx, max_idx);

//        EXPECT_EQ(expected_index, max_idx[1]);
//        EXPECT_NE(min_idx[1], max_idx[1]);

        angle += 90.;
        expected_index++;
    //}
}

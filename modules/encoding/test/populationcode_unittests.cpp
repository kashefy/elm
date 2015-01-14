#include "encoding/populationcode.h"

#include <opencv2/highgui.hpp>

#include "core/cv/mat_utils_inl.h"
#include "core/layerconfig.h"
#include "core/signal.h"
#include "encoding/orientation.h"
#include "io/synth.h"
#include "layers/layerfactory.h"
#include "ts/ts.h"

using std::string;
using cv::Mat1f;
using namespace sem;

namespace {

class SoftMaxPopulationCodeTest : public testing::Test
{
protected:
    static void SetUpTestCase()
    {
        const int RADIUS=9;
        SIZE_ = RADIUS*2+1;
        const float SIGMA = 3;
        const float _LAMBDA = 10;   //CV_PI;
        const float _GAMMA = 0.02;   //10;
        const float PS = 0;         //CV_PI*0.5;

        const float THETA_STOP=CV_PI;
        THETA_STEP_ = 90.*CV_PI/180.;
        Mat1f theta_range = ARange_<float>(0.f, THETA_STOP, THETA_STEP_);

        // Mat1f to VecF
        const float* p = theta_range.ptr<float>(0);
        VecF theta(p, p+theta_range.cols);

        kernels_ = GaborFilterBank::CreateKernels(RADIUS, SIGMA, theta, _LAMBDA, _GAMMA, PS);
        NB_KERNELS_ = static_cast<int>(kernels_.size());

        gabors_.reset(new GaborFilterBank());
        dynamic_cast<GaborFilterBank*>(gabors_.get())->Reset(RADIUS, SIGMA, theta_range, _LAMBDA, _GAMMA, PS);
    }

    void virtual SetUp()
    {
    }

    SoftMaxPopulationCode to_; ///< test object
    Mat1f in_;
    static int SIZE_;
    static int NB_KERNELS_;
    static VecMat1f kernels_;
    static float THETA_STEP_;

    static std::unique_ptr<base_FilterBank> gabors_; ///< filter bank to hold gabor filters
};
VecMat1f SoftMaxPopulationCodeTest::kernels_;
int SoftMaxPopulationCodeTest::SIZE_;
int SoftMaxPopulationCodeTest::NB_KERNELS_;
float SoftMaxPopulationCodeTest::THETA_STEP_;
std::unique_ptr<base_FilterBank> SoftMaxPopulationCodeTest::gabors_;

TEST_F(SoftMaxPopulationCodeTest, PopCode_dims)
{
    in_ = Mat1f::zeros(SIZE_, SIZE_);
    to_.State(in_, kernels_);
    cv::Mat pop_code = to_.PopCode();
    EXPECT_GT(static_cast<int>(in_.total())*NB_KERNELS_, 0);
    EXPECT_MAT_DIMS_EQ(pop_code, cv::Size(in_.total()*NB_KERNELS_, 1));
    EXPECT_MAT_TYPE(pop_code, CV_32F);
}

TEST_F(SoftMaxPopulationCodeTest, PopCode_zeros)
{
    in_ = Mat1f::zeros(SIZE_, SIZE_);
    to_.State(in_, kernels_);
    cv::Mat pop_code = to_.PopCode();
    EXPECT_MAT_EQ(pop_code, Mat1f::zeros(1, in_.total()*NB_KERNELS_));
}

TEST_F(SoftMaxPopulationCodeTest, PopCode_uniform)
{
    in_ = Mat1f::ones(5, 5);

    VecMat1f identity_kernels;
    const int NB_ID_KERNELS=3;
    for(int i=0; i<NB_ID_KERNELS; i++) {
        identity_kernels.push_back(Mat1f::ones(3, 3));
    }

    to_.State(in_, identity_kernels);

    const int N=1e3;
    Mat1f counts = Mat1f::zeros(1, static_cast<int>(in_.total())*NB_ID_KERNELS);

    for(int i=0; i<N; i++) {

        cv::Mat pop_code = to_.PopCode();
        EXPECT_MAT_DIMS_EQ(pop_code, counts);
        EXPECT_EQ(cv::sum(pop_code)(0), in_.total())
                << "Since input and kernels are all-ones,"
                << "we expect one non-zero population code per input element.";
        cv::add(pop_code, counts, counts);
    }

    EXPECT_FLOAT_EQ(cv::sum(counts)(0), N*in_.total())
            << "Not all samples accounted for."
            << "Input elements are sampled independently.";

    counts /= static_cast<float>(N);

    cv::Mat m, s;
    cv::meanStdDev(counts, m, s);

    EXPECT_FLOAT_EQ(1./static_cast<double>(NB_ID_KERNELS), m.at<double>(0));
    EXPECT_LE(s.at<double>(0), 0.02);
}

TEST_F(SoftMaxPopulationCodeTest, PopCode_orientation)
{
    SynthBars bars;
    bars.Reset(28, 28, 1);

    float angle = 0.f;
    int expected_index = 0;
    while(angle < 180.f) {

        cv::Mat img;
        bars.Draw(angle, img);
        img.convertTo(in_, CV_32FC1, 1./255.);
        to_.State(in_, kernels_);

        const int N=10;
        Mat1f counts = Mat1f::zeros(1, static_cast<int>(in_.total())*NB_KERNELS_);
        for(int i=0; i<N; i++) {

            cv::Mat pop_code = to_.PopCode();
            EXPECT_MAT_DIMS_EQ(pop_code, counts);
            EXPECT_LE(cv::sum(pop_code)(0), in_.total())
                    << "Encountered oversampling for same input element.";
            cv::add(pop_code, counts, counts);
        }

        counts = counts.reshape(1, in_.total());

        // for which did the bar respond the most?
        cv::Mat1i col_sums(1, NB_KERNELS_);
        for(int i=0; i<counts.cols; i++) {

            col_sums(i) = static_cast<int>(cv::sum(counts.col(i))(0));
        }

        double min_val;
        int min_idx[2] = {-1, -1};
        int max_idx[2] = {-1, -1};
        cv::minMaxIdx(col_sums, &min_val, 0, min_idx, max_idx);

        EXPECT_EQ(expected_index, max_idx[1]);
        EXPECT_NE(min_idx[1], max_idx[1]);

        angle += THETA_STEP_/CV_PI*180.;
        expected_index++;
    }
}

TEST_F(SoftMaxPopulationCodeTest, PopCode_filter_bank_orientation)
{
    SynthBars bars;
    bars.Reset(28, 28, 1);

    float angle = 0.f;
    int expected_index = 0;

    while(angle < 180.f) {

        cv::Mat img;
        bars.Draw(angle, img);
        img.convertTo(in_, CV_32FC1, 1./255.);
        to_.State(in_, gabors_);

        const int N=10;
        Mat1f counts = Mat1f::zeros(1, static_cast<int>(in_.total())*NB_KERNELS_);
        for(int i=0; i<N; i++) {

            cv::Mat pop_code = to_.PopCode();
            EXPECT_MAT_DIMS_EQ(pop_code, counts);
            EXPECT_LE(cv::sum(pop_code)(0), in_.total())
                    << "Encountered oversampling for same input element.";
            cv::add(pop_code, counts, counts);
        }

        counts = counts.reshape(1, in_.total());

        // for which did the bar respond the most?
        cv::Mat1i col_sums(1, NB_KERNELS_);
        for(int i=0; i<counts.cols; i++) {

            col_sums(i) = static_cast<int>(cv::sum(counts.col(i))(0));
        }

        double min_val;
        int min_idx[2] = {-1, -1};
        int max_idx[2] = {-1, -1};
        cv::minMaxIdx(col_sums, &min_val, 0, min_idx, max_idx);

        EXPECT_EQ(expected_index, max_idx[1]);
        EXPECT_NE(min_idx[1], max_idx[1]);

        angle += THETA_STEP_/CV_PI*180.;
        expected_index++;
    }
}

} // annonymous namespace for test fixtures

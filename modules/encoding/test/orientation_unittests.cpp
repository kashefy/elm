#include "encoding/orientation.h"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "core/defs.h"
#include "core/exception.h"
#include "core/mat_utils.h"
#include "io/synth.h"
#include "io/readmnist.h"
#include "ts/ts.h"

using namespace cv;
using namespace sem;

const int RADIUS    = 9;        ///< kernel radius
const float SIGMA   = 3;        ///< sigma of gaussian envelope
const float _LAMBDA = 10;       ///< sinusoid wavelength
const float _GAMMA  = 0.02;    ///< gamma
const float PS      = 0;        ///< phase shift

TEST(GaborKernelTest, KernelSize)
{
    EXPECT_THROW(GaborFilterBank::CreateKernel(-10, 1., 0., 1., 1., 0.), ExceptionValueError);
    EXPECT_THROW(GaborFilterBank::CreateKernel(0, 1., 0., 1., 1., 0.), ExceptionValueError);

    for(int radius=1; radius<10; radius+=3) {

        Mat kernel = GaborFilterBank::CreateKernel(radius, 1., 0., 1., 1., 0.);
        EXPECT_MAT_DIMS_EQ(kernel, Mat(radius*2+1, radius*2+1, CV_32F));
        EXPECT_EQ(kernel.rows, kernel.cols) << "kernel not a square.";
    }
}

TEST(GaborKernelTest, KernelMat)
{
    Mat kernel = GaborFilterBank::CreateKernel(3, 1., 0., 1., 1., 0.);
    EXPECT_MAT_TYPE(kernel, CV_32F);
}

TEST(GaborKernelTest, KernelRotated)
{
    for(float angle=0.f; angle<=360.f; angle+=45.f) {

        Mat kernel0 = GaborFilterBank::CreateKernel(RADIUS, SIGMA, angle*CV_PI/180.0f, _LAMBDA, _GAMMA, PS);
        Mat kernel90 = GaborFilterBank::CreateKernel(RADIUS, SIGMA, (angle+90)*CV_PI/180.0f, _LAMBDA, _GAMMA, PS);
        Mat kernel180 = GaborFilterBank::CreateKernel(RADIUS, SIGMA, (angle+180)*CV_PI/180.0f, _LAMBDA, _GAMMA, PS);

        EXPECT_MAT_NEAR(kernel0, kernel180, 1e-7);

        flip(kernel90.t(), kernel90, 0);

        EXPECT_MAT_NEAR(kernel0, kernel90, 1e-7);
    }
}

TEST(GaborKernelTest, Response)
{
    for(float angle=0.f; angle<=180.f; angle+=45.f) {

        Mat kernel = GaborFilterBank::CreateKernel(RADIUS, SIGMA, angle*CV_PI/180.0f, _LAMBDA, _GAMMA, PS);

        SynthBars bars;
        bars.Reset(28, 28, 1);

        Mat img, response;
        bars.Draw(angle, img);
        img.convertTo(img, CV_32F, 1./255.);
        filter2D(img, response, -1, kernel);

        EXPECT_MAT_DIMS_EQ(img, response);
        EXPECT_MAT_TYPE(response, CV_32F);
        EXPECT_EQ(img.channels(), response.channels());

        // check response has a line through the center
        if(angle == 0 || angle == 180) {

            int mid = response.rows/2;
            EXPECT_GT(sum(response.rowRange(mid-2, mid+2))(0),
                      sum(response.rowRange(0, 4))(0));
            EXPECT_GT(sum(response.rowRange(mid-2, mid+2))(0),
                      sum(response.rowRange(response.rows-4, response.rows))(0));
        }
        else if(angle == 90) {

            int mid = response.cols/2;
            EXPECT_GT(sum(response.colRange(mid-2, mid+2))(0),
                      sum(response.colRange(0, 4))(0));
            EXPECT_GT(sum(response.colRange(mid-2, mid+2))(0),
                      sum(response.colRange(response.cols-4, response.cols))(0));
        }

        // check response of bar perpendicular to kernel orientation
        Mat img_orth, response_orth;
        bars.Draw(angle, img_orth);
        img.convertTo(img_orth, CV_32F, 1./255.);
        filter2D(img_orth, response_orth, -1, kernel);

        EXPECT_GT(sum(response)(0), sum(response_orth)(0));
    }
}

TEST(GaborKernelTest, DISABLED_DisplayKernel)
{
    for(float angle=0.f; angle<=360.f; angle+=45.f) {

        const float theta = angle * CV_PI / 180.0f;
        Mat kernel = GaborFilterBank::CreateKernel(RADIUS, SIGMA, theta, _LAMBDA, _GAMMA, PS);

        std::stringstream s;
        s << "angle="<<angle;
        imshow(s.str().c_str(), ConvertTo8U(kernel));
        std::cout<<s.str()<<kernel<<std::endl;
        waitKey(0.5);
    }
}

TEST(GaborKernelTest, FilterBankVector)
{
    const size_t N=9;

    float angle=-SEM_PI2;
    VecF theta;

    for(size_t i=0; i<N; i++) {

        VecMat1f filter_bank = GaborFilterBank::CreateKernels(RADIUS, SIGMA, theta, _LAMBDA, _GAMMA, PS);

        EXPECT_EQ(filter_bank.size(), i);

        for(size_t j=0; j<i; j++) {

            EXPECT_MAT_DIMS_EQ(filter_bank[j], Size(RADIUS*2+1, RADIUS*2+1));
            EXPECT_MAT_TYPE(filter_bank[j], CV_32F);
        }

        theta.push_back(angle);
        angle += SEM_PI2;
    }
}

TEST(GaborKernelTest, FilterBankKernels)
{
    const float THETA_STOP=CV_PI;
    const float THETA_STEP=30.*CV_PI/180.;
    Mat1f theta_range = ARange_<float>(0.f, THETA_STOP, THETA_STEP);

    // Mat1f to VecF
    const float* p = theta_range.ptr<float>(0);
    VecF theta(p, p+theta_range.cols);

    VecMat1f filter_bank = GaborFilterBank::CreateKernels(RADIUS, SIGMA, theta, _LAMBDA, _GAMMA, PS);
    EXPECT_EQ(filter_bank.size(), theta_range.total());

    float angle=0.f;
    int i=0;
    while(angle < THETA_STOP) {

        Mat1f kernel = GaborFilterBank::CreateKernel(RADIUS, SIGMA, angle, _LAMBDA, _GAMMA, PS);
        EXPECT_MAT_EQ(filter_bank[i], kernel);
        i++;
        angle += THETA_STEP;
    }

    // test orthogonal kernels
    EXPECT_FALSE( Equal(filter_bank[0], filter_bank[3]) );
    Mat1f kernel90 = filter_bank[0];
    flip(kernel90.t(), kernel90, 0);
    EXPECT_MAT_EQ( kernel90, filter_bank[0] ) << "Not identical after 90 deg rotation";
}

/**
 * @brief Test gabor filter banks
 */
class GaborFilterBankTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        to_ = GaborFilterBank();

        const float THETA_STOP=CV_PI;
        const float THETA_STEP=45.*CV_PI/180.;
        theta_range_ = ARange_<float>(0.f, THETA_STOP, THETA_STEP);

        to_.Reset(RADIUS, SIGMA, theta_range_, _LAMBDA, _GAMMA, PS);

        kernels_ = GaborFilterBank::CreateKernels(RADIUS, SIGMA, theta_range_, _LAMBDA, _GAMMA, PS);

    }

    GaborFilterBank to_;    ///< test object
    VecMat1f kernels_;      ///< reference kernels
    Mat1f theta_range_;      ///< range of kernel orientations
};

/**
 * @brief test new kernels and their properties after a reset
 */
TEST_F(GaborFilterBankTest, ResetKernels)
{
    for(int radius=9; radius<=15; radius+=3) {

        for(float delta=30; delta<=90; delta+=30) {

            const float THETA_STOP=CV_PI;
            const float THETA_STEP=delta*CV_PI/180.f;
            Mat1f theta_range = ARange_<float>(0.f, THETA_STOP, THETA_STEP);
            VecMat1f expected_kernels = GaborFilterBank::CreateKernels(radius, SIGMA, theta_range, _LAMBDA, _GAMMA, PS);

            to_.Reset(radius, SIGMA, theta_range, _LAMBDA, _GAMMA, PS);

            VecMat1f actual_kernels = to_.Kernels();

            // check no. of kernels
            ASSERT_GT(theta_range.total(), size_t(0)) << "Revise test to be more effective. Need size > 0.";
            EXPECT_EQ(expected_kernels.size(), actual_kernels.size()) << "wrong no. of kernels";
            EXPECT_EQ(theta_range.total(), actual_kernels.size()) << "wrong no. of kernels";

            // check kernel values
            for(size_t k=0; k<expected_kernels.size(); k++) {

                EXPECT_MAT_EQ(expected_kernels[k], actual_kernels[k]) << "Kernel mismatch";
            }
        }
    }
}

/**
 * @brief test size() getter method, should match no. of kernels
 */
TEST_F(GaborFilterBankTest, SizeGetter)
{
    for(float delta=30; delta<=90; delta+=30) {

        const float THETA_STOP=CV_PI;
        const float THETA_STEP=delta*CV_PI/180.f;
        Mat1f theta_range = ARange_<float>(0.f, THETA_STOP, THETA_STEP);

        to_.Reset(RADIUS, SIGMA, theta_range, _LAMBDA, _GAMMA, PS);

        VecMat1f actual_kernels = to_.Kernels();

        ASSERT_GT(theta_range.total(), size_t(0)) << "Revise test to be more effective. Need size > 0.";
        EXPECT_EQ(theta_range.total(), actual_kernels.size()) << "wrong no. of kernels";
        EXPECT_EQ(theta_range.total(), to_.size()) << "incorrect filter bank size";
        EXPECT_EQ(actual_kernels.size(), to_.size()) << "size of this filter bank does not match no. kernels.";
    }
}

/**
 * @brief Test computing response from gabor filter bank
 * - define the stimulus as an oriented bar that matches the preferred orientation of one of the kernels
 * - check that highest response originates from expect kernel
 * - recreate the kernel
 * - match response magnitude of filter bank with response to standalone kernel
 * - iterate to next orientation
 */
TEST_F(GaborFilterBankTest, Compute)
{
    for(int size=28; size<100; size*=2) {

        SynthBars bars;
        bars.Reset(size, size, 1);

        VecMat1f kernels = to_.Kernels();

        for(size_t i=0; i<theta_range_.total(); i++) {

            float theta = theta_range_(i);
            Mat img, stimulus;
            bars.Draw(theta*180.f/CV_PI, img);
            img.convertTo(stimulus, CV_32FC1, 1./255.);

            VecMat1f response_actual = to_.Compute(stimulus);

            EXPECT_EQ(kernels.size(), response_actual.size()) << "response size does not match no. of kernels.";

            // get index of largest response through linear search
            size_t j_max = -1;
            float max_response = -1.;
            for(size_t j=0; j<response_actual.size(); j++) {

                float j_sum = sum(response_actual[j])(0);
                if(j_sum > max_response) {

                    j_max = j;
                    max_response = j_sum;
                }
            }
            EXPECT_EQ(i, j_max) << "Unexpected index for kernel yielding highest response";

            Mat1f response_single_expected;
            Mat kernel = GaborFilterBank::CreateKernel(RADIUS, SIGMA, theta, _LAMBDA, _GAMMA, PS);

            filter2D(stimulus, response_single_expected, -1, kernel, Point(-1, -1), 0, cv::BORDER_REPLICATE);
            pow(response_single_expected, 2., response_single_expected);

            EXPECT_MAT_EQ(kernel, kernels[i]) << "Kernels do not match";

            EXPECT_FLOAT_EQ(sum(response_single_expected)(0), max_response) << "Unexpected sum of response";
        }
    }
}

TEST_F(GaborFilterBankTest, ElementResponse)
{
    const int SIZE=28;
    SynthBars bars;
    bars.Reset(SIZE, SIZE, 1);

    Mat img, stimulus;
    bars.Draw(45.f, img);
    img.convertTo(stimulus, CV_32FC1, 1./255.);

    VecMat1f response = to_.Compute(stimulus);

    const int N=100;
    for(int i=0; i<N; i++) {

        int row = abs(randu<int>()) % SIZE;
        int col = abs(randu<int>()) % SIZE;
        Mat1f el_response_expected(1, static_cast<int>(response.size()), 0.f);
        for(size_t k=0; k<response.size(); k++) {

            el_response_expected(k) = response[k](row, col);
        }

        Mat1f el_response_actual = to_.ElementResponse(row, col);

        EXPECT_MAT_DIMS_EQ(el_response_actual, Size(static_cast<int>(response.size()), 1)) << "Expecting element response in a row matrix";

        EXPECT_MAT_EQ(el_response_expected, el_response_actual) << "Unexpected element response";
    }
}

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
    const int RADIUS=9;
    const float sigma = 3;
    const float _lambda = 10;//CV_PI;
    const float gamma = 0.02;//10;
    const float ps = 0;//CV_PI*0.5;

    for(float angle=0.f; angle<=360.f; angle+=45.f) {

        Mat kernel0 = GaborFilterBank::CreateKernel(RADIUS, sigma, angle*CV_PI/180.0f, _lambda, gamma, ps);
        Mat kernel90 = GaborFilterBank::CreateKernel(RADIUS, sigma, (angle+90)*CV_PI/180.0f, _lambda, gamma, ps);
        Mat kernel180 = GaborFilterBank::CreateKernel(RADIUS, sigma, (angle+180)*CV_PI/180.0f, _lambda, gamma, ps);

        EXPECT_MAT_NEAR(kernel0, kernel180, 1e-7);

        flip(kernel90.t(), kernel90, 0);

        EXPECT_MAT_NEAR(kernel0, kernel90, 1e-7);
    }
}

TEST(GaborKernelTest, Response)
{
    const int RADIUS=9;
    const float sigma = 3;
    const float _lambda = 10;//CV_PI;
    const float gamma = 0.02;//10;
    const float ps = 0;//CV_PI*0.5;

    for(float angle=0.f; angle<=180.f; angle+=45.f) {

        Mat kernel = GaborFilterBank::CreateKernel(RADIUS, sigma, angle*CV_PI/180.0f, _lambda, gamma, ps);

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
    const int RADIUS=9;
    const float SIGMA = 3;
    const float _LAMBDA = 10;   //CV_PI;
    const float GAMMA = 0.02;   //10;
    const float PS = 0;         //CV_PI*0.5;

    for(float angle=0.f; angle<=360.f; angle+=45.f) {

        const float theta = angle * CV_PI / 180.0f;
        Mat kernel = GaborFilterBank::CreateKernel(RADIUS, SIGMA, theta, _LAMBDA, GAMMA, PS);

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
    const int RADIUS=9;
    const float SIGMA = 3;
    const float _LAMBDA = 10;   //CV_PI;
    const float GAMMA = 0.02;   //10;
    const float PS = 0;         //CV_PI*0.5;

    float angle=-SEM_PI2;
    VecF theta;

    for(size_t i=0; i<N; i++) {

        VecMat1f filter_bank = GaborFilterBank::CreateKernels(RADIUS, SIGMA, theta, _LAMBDA, GAMMA, PS);

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
    const int RADIUS=9;
    const float SIGMA = 3;
    const float _LAMBDA = 10;   //CV_PI;
    const float GAMMA = 0.02;   //10;
    const float PS = 0;         //CV_PI*0.5;

    const float THETA_STOP=CV_PI;
    const float THETA_STEP=30.*CV_PI/180.;
    Mat1f theta_range = ARange_<float>(0.f, THETA_STOP, THETA_STEP);

    // Mat1f to VecF
    const float* p = theta_range.ptr<float>(0);
    VecF theta(p, p+theta_range.cols);

    VecMat1f filter_bank = GaborFilterBank::CreateKernels(RADIUS, SIGMA, theta, _LAMBDA, GAMMA, PS);
    EXPECT_EQ(filter_bank.size(), theta_range.total());

    float angle=0.f;
    int i=0;
    while(angle < THETA_STOP) {

        Mat1f kernel = GaborFilterBank::CreateKernel(RADIUS, SIGMA, angle, _LAMBDA, GAMMA, PS);
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

    }
};


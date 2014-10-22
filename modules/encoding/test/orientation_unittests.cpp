#include "encoding/orientation.h"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "core/exception.h"
#include "core/mat_utils.h"
#include "io/synth.h"
#include "io/readmnist.h"
#include "ts/ts.h"

using namespace cv;
using namespace sem;

class GaborTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {

    }
};

TEST_F(GaborTest, KernelSize)
{
    EXPECT_THROW(GaborKernel(-10, 1., 0., 1., 1., 0.), ExceptionValueError);
    EXPECT_THROW(GaborKernel(0, 1., 0., 1., 1., 0.), ExceptionValueError);

    for(int radius=1; radius<10; radius+=3) {

        Mat kernel = GaborKernel(radius, 1., 0., 1., 1., 0.);
        EXPECT_MAT_DIMS_EQ(kernel, Mat(radius*2+1, radius*2+1, CV_32F));
        EXPECT_EQ(kernel.rows, kernel.cols) << "kernel not a square.";
    }
}

TEST_F(GaborTest, KernelMat)
{
    Mat kernel = GaborKernel(3, 1., 0., 1., 1., 0.);
    EXPECT_MAT_TYPE(kernel, CV_32F);
}

TEST_F(GaborTest, KernelRotated)
{
    const int RADIUS=9;
    const float sigma = 3;
    const float _lambda = 10;//CV_PI;
    const float gamma = 0.02;//10;
    const float ps = 0;//CV_PI*0.5;

    for(float angle=0.f; angle<=360.f; angle+=45.f) {

        Mat kernel0 = GaborKernel(RADIUS, sigma, angle*CV_PI/180.0f, _lambda, gamma, ps);
        Mat kernel90 = GaborKernel(RADIUS, sigma, (angle+90)*CV_PI/180.0f, _lambda, gamma, ps);
        Mat kernel180 = GaborKernel(RADIUS, sigma, (angle+180)*CV_PI/180.0f, _lambda, gamma, ps);

        EXPECT_MAT_NEAR(kernel0, kernel180, 1e-7);

        flip(kernel90.t(), kernel90, 0);

        EXPECT_MAT_NEAR(kernel0, kernel90, 1e-7);
    }
}

TEST_F(GaborTest, Response)
{
    const int RADIUS=9;
    const float sigma = 3;
    const float _lambda = 10;//CV_PI;
    const float gamma = 0.02;//10;
    const float ps = 0;//CV_PI*0.5;

    for(float angle=0.f; angle<=180.f; angle+=45.f) {

        Mat kernel = GaborKernel(RADIUS, sigma, angle*CV_PI/180.0f, _lambda, gamma, ps);

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

TEST_F(GaborTest, DISABLED_DisplayKernel)
{
    const int RADIUS=9;
    const float sigma = 3;
    const float _lambda = 10;//CV_PI;
    const float gamma = 0.02;//10;
    const float ps = 0;//CV_PI*0.5;

    for(float angle=0.f; angle<=360.f; angle+=45.f) {

        const float theta = angle * CV_PI / 180.0f;
        Mat kernel = GaborKernel(RADIUS, sigma, theta, _lambda, gamma, ps);

        std::stringstream s;
        s << "angle="<<angle;
        imshow(s.str().c_str(), ConvertTo8U(kernel));
        std::cout<<s.str()<<kernel<<std::endl;
        waitKey(0.5);
    }
}


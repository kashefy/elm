#include "encoding/orientation.h"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "core/exception.h"
#include "io/synth.h"
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

TEST_F(GaborTest, DISABLED_Kernel)
{
    const int RADIUS=9;
    const float sigma = 10;//3;
    //const float theta = 90.f;
    const float theta = 90.f * CV_PI / 180.0f;
    const float _lambda = 10;//CV_PI;
    const float gamma = 0.02;//10;
    const float ps = 0;//CV_PI*0.5;


    Mat kernel = GaborKernel(RADIUS, sigma, theta, _lambda, gamma, ps);
    //Mat kernel = GaborKernel(RADIUS, 3, CV_PI/2.0, CV_PI, 10.0, CV_PI*0.5);

    std::cout<<"k"<<kernel<<std::endl;

    kernel /= sum(kernel)(0);

    SynthBars x;
    x.Reset(28, 28, 2);
    Mat img, in, label;
    x.Next(img, label);
    img.convertTo(in, CV_32F, 1./255.);
    //std::cout<<"i"<<in<<std::endl;
    //std::cout<<label<<std::endl;
    Mat response;
    filter2D(in, response, -1, kernel);

    std::cout<<"r"<<response<<std::endl;

    int min_idx[2] = {-1, -1};
    int max_idx[2] = {-1, -1};
    double min_val, max_val;
    minMaxIdx(response, &min_val, &max_val, min_idx, max_idx);

    response -= min_val;

    minMaxIdx(response, &min_val, &max_val, min_idx, max_idx);

    response /= max_val;

    minMaxIdx(response, &min_val, &max_val, min_idx, max_idx);
    std::cout<<"min "<<min_val<<std::endl;
    std::cout<<"max "<<max_val<<std::endl;

    Mat ru;
    response.convertTo(ru, CV_8UC1, 255.);
    imshow("ru", ru);
    waitKey();

    return;
//std::cout<<kernel<<std::endl;

    Mat viz, m, s;
    cv::meanStdDev(kernel, m, s);
    //std::cout<<m<<std::endl;
    //kernel /= m;
    //kernel *= 255/4;

    kernel -= min_val;

    minMaxIdx(kernel, &min_val, &max_val, min_idx, max_idx);
    kernel /= cv::sum(kernel)(0);

    minMaxIdx(kernel, &min_val, &max_val, min_idx, max_idx);
    kernel /= max_val;
    kernel *= 255;

    minMaxIdx(kernel, &min_val, &max_val, min_idx, max_idx);

    std::cout<<"min "<<min_val<<std::endl;
    std::cout<<"max "<<max_val<<std::endl;


    cv::meanStdDev(kernel, m, s);
    std::cout<<m<<std::endl;

    kernel.convertTo(viz, CV_8U);

    imshow("x", viz(Rect(280, 280, 40, 40 )));
    waitKey();
}


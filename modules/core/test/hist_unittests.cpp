#include "core/hist.h"

#include <vector>

#include <opencv2/imgproc.hpp>

#include "core/exception.h"
#include "core/cv/mat_utils.h"
#include "ts/ts.h"

using namespace cv;
using namespace sem;

/** @brief class for testing single-channe histogram wrapper
 */
class HistCh1Test : public ::testing::Test
{
protected:
    virtual void SetUp() {

        to_ = Hist1Ch();
    }

    Hist1Ch to_; ///< test object
};

/**
 * @brief test parameter validation
 */
TEST_F(HistCh1Test, InvalidSize)
{
    EXPECT_THROW(to_.Reconfigure(-1, std::make_pair(0.f, 10.f), true, false), ExceptionValueError) << "A histogram with negative size doesn't make sense.";
    EXPECT_THROW(to_.Reconfigure(-2, std::make_pair(0.f, 10.f), true, false), ExceptionValueError) << "A histogram with negative size doesn't make sense.";
    EXPECT_THROW(to_.Reconfigure(-100, std::make_pair(0.f, 10.f), true, false), ExceptionValueError) << "A histogram with negative size doesn't make sense.";

    // or let size==0 throw?
    EXPECT_NO_THROW(to_.Reconfigure(0, std::make_pair(0.f, 10.f), true, false));
    EXPECT_NO_THROW(to_.Reconfigure(100, std::make_pair(0.f, 10.f), true, false));
}

/**
 * @brief We're starting with only supporting single channel input.
 * This may change afterwards, so this test will need to be updated when this happens.
 */
TEST_F(HistCh1Test, TooManyChannels)
{
    EXPECT_THROW(to_.Compute(Mat2f::zeros(2, 2)), ExceptionBadDims) << "Read to support multiple channels?";
    EXPECT_THROW(to_.Compute(Mat3f::zeros(2, 2)), ExceptionBadDims) << "Read to support multiple channels?";
    EXPECT_THROW(to_.Compute(Mat::ones(2, 2, CV_32FC4)), ExceptionBadDims) << "Read to support multiple channels?";

    EXPECT_NO_THROW(to_.Compute(Mat1f::zeros(2, 2))) << "Failed to support single-channel input.";
}

/**
 * @brief test with input that ensures 1 element per bin
 */
TEST_F(HistCh1Test, OnePerBin)
{
    Mat1f in = ARange_<float>(0.f, 3.f, 1.f);
    to_.Reconfigure(3, std::make_pair(0.f, 3.f), true, false);
    EXPECT_MAT_EQ(Mat1f::ones(1, in.total()), to_.Compute(in));
    EXPECT_MAT_EQ(Mat1f::ones(1, in.total()), to_.Compute(in.t())) << "Row-matrix expected even when input transposed.";
}

/**
 * @brief test with input that lies outside bin ranges
 */
TEST_F(HistCh1Test, OutsideBinRange)
{
    const int NB_BINS=3;
    const float LOWER_BOUND=1.f;
    const float UPPER_BOUND=10.f;
    to_.Reconfigure(NB_BINS, std::make_pair(LOWER_BOUND, UPPER_BOUND), true, false);
    EXPECT_MAT_EQ(Mat1f::zeros(1, NB_BINS), to_.Compute(Mat1f::zeros(1, 10)));
    EXPECT_MAT_EQ(Mat1f::zeros(1, NB_BINS), to_.Compute(Mat1f(1, NB_BINS, UPPER_BOUND+0.01))) << "All input elements above upper bound, outside of all bins";
    EXPECT_MAT_EQ(Mat1f::zeros(1, NB_BINS), to_.Compute(Mat1f(1, NB_BINS, LOWER_BOUND-0.01))) << "All input elements below lower bound, outside of all bins";
}

TEST_F(HistCh1Test, HistType)
{
    EXPECT_MAT_TYPE(to_.Compute(Mat::ones(1, 2, CV_32FC1)), CV_32FC1);

    to_.Reconfigure(2, std::make_pair<float, float>(0.f, 255.f), true, false);
    EXPECT_MAT_TYPE(to_.Compute(Mat::ones(1, 2, CV_8UC1)), CV_32FC1);
    EXPECT_MAT_TYPE(to_.Compute(Mat::ones(1, 2, CV_16UC1)), CV_32FC1);

    // unsupported matrix types
    EXPECT_THROW(to_.Compute(Mat::ones(1, 2, CV_8SC1)), ExceptionTypeError);
    EXPECT_THROW(to_.Compute(Mat::ones(1, 2, CV_16SC1)), ExceptionTypeError);
    EXPECT_THROW(to_.Compute(Mat::ones(1, 2, CV_32SC1)), ExceptionTypeError);
    EXPECT_THROW(to_.Compute(Mat::ones(1, 2, CV_64FC1)), ExceptionTypeError);
}

/**
* @brief test histogram caluclation with data that will yield delta shaped histograms
* "all eggs in one basket"
*/
TEST_F(HistCh1Test, AllInOne)
{
    const int NB_BINS=10;
    const int LEN=100;
    const float LOWER_BOUND=0.f;
    const float UPPER_BOUND=10.f;
    to_.Reconfigure(NB_BINS, std::make_pair(LOWER_BOUND, UPPER_BOUND), true, false);

    for(int i_bin=0; i_bin<NB_BINS; i_bin++) {

        Mat1f expected_hist = Mat1f::zeros(1, NB_BINS);
        expected_hist(i_bin) = LEN;
        EXPECT_MAT_EQ(expected_hist, to_.Compute(Mat1f(1, LEN, i_bin)));
    }
}

/**
 * @brief test that bin edges are treated as left-closed, right-open intervals.
 * Therefore, the lower bound should be inclusive.
 */
TEST_F(HistCh1Test, LowerBoundInclusive)
{
    const int NB_BINS=10;
    const int LEN=100;
    const float LOWER_BOUND=0.f;
    const float UPPER_BOUND=10.f;
    to_.Reconfigure(NB_BINS, std::make_pair(LOWER_BOUND, UPPER_BOUND), true, false);

    Mat1f expected_hist = Mat1f::zeros(1, NB_BINS);
    expected_hist(0) = LEN;
    EXPECT_MAT_EQ(expected_hist, to_.Compute(Mat1f(1, LEN, LOWER_BOUND)));
}

/**
 * @brief test that bin edges are treated as left-closed, right-open intervals
 * Therefore, the upper bound should be exclusive and not register in the histogram
 */
TEST_F(HistCh1Test, UpperBoundExclusive)
{
    const int NB_BINS=10;
    const int LEN=100;
    const float LOWER_BOUND=0.f;
    const float UPPER_BOUND=10.f;
    to_.Reconfigure(NB_BINS, std::make_pair(LOWER_BOUND, UPPER_BOUND), true, false);

    EXPECT_MAT_EQ(Mat1f::zeros(1, NB_BINS), to_.Compute(Mat1f(1, LEN, UPPER_BOUND)));
}

TEST_F(HistCh1Test, Hist)
{
  /// Separate the image in 3 places ( B, G and R )
  std::vector<Mat> bgr_planes;

  Mat1f b(1, 1000);
  randn(b, 0, 1);
  Mat1f g(1, 1000);
  randn(g, 0, 3);
  Mat1f r(1, 1000);
  randn(r, 4, 1);

  bgr_planes.push_back(b);
  bgr_planes.push_back(g);
  bgr_planes.push_back(r);

  /// Establish the number of bins
  int histSize = 20;

  /// Set the ranges ( for B,G,R) )
  float range[] = { -10, 10 } ;
  const float* histRange = { range };

  bool uniform = true;
  bool accumulate = false;

  Mat b_hist, g_hist, r_hist;

  /// Compute the histograms:
  calcHist( &bgr_planes[0], 1, 0, Mat(), b_hist, 1, &histSize, &histRange, uniform, accumulate );
  calcHist( &bgr_planes[1], 1, 0, Mat(), g_hist, 1, &histSize, &histRange, uniform, accumulate );
  calcHist( &bgr_planes[2], 1, 0, Mat(), r_hist, 1, &histSize, &histRange, uniform, accumulate );

  std::cout<<b_hist.t()<<std::endl;
  std::cout<<g_hist.t()<<std::endl;
  std::cout<<r_hist.t()<<std::endl;
}

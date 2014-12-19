#include "core/hist.h"

#include <vector>

#include <opencv2/imgproc.hpp>

#include "ts/ts.h"

using namespace cv;

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

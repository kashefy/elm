#ifndef SEM_CORE_CV_NEIGHBORHOOD_H_
#define SEM_CORE_CV_NEIGHBORHOOD_H_

#include <opencv2/core.hpp>

namespace sem
{

/**
  * @brief Calculate neighborhood variance
  *
  * Border elements are zero-padded
  * \see http://docs.opencv.org/doc/tutorials/imgproc/imgtrans/copyMakeBorder/copyMakeBorder.html
  *
  * @param[in] source matrix
  * @param[in] neighborhood radius
  * @param[out] matrix with neighborhood mean around each element
  * @param[out] matrix with neighborhood variance around each element
  * @param[in] border type - \see OpenCV's copyMakeBorder() or borderInterpolate() for details. default is cv::BORDER_REPLICATE
  * @param[in] value - Border value if border type==BORDER_CONSTANT
  */
void NeighMeanVar(const cv::Mat1f& m, int radius, cv::Mat1f &neigh_mean, cv::Mat1f &neigh_var,
                  int border_type=cv::BORDER_REPLICATE, const cv::Scalar &value=cv::Scalar());

} // namespace sem

#endif // SEM_CORE_CV_NEIGHBORHOOD_H_

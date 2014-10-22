#ifndef SEM_CORE_MAT_UTILS_H_
#define SEM_CORE_MAT_UTILS_H_

#include <string>
#include "core/typedefs.h"

namespace sem {

/**
 * @brief Converts matrix to 8U and scales elements
 * so that min=0 and max=255.
 * If all input elements are identical, yield matrix of zeros
 *
 * Useful for preparing matrix to use with highgui's cv::imshow()
 *
 * @param src
 * @return Mat_<uchar>
 */
cv::Mat_<unsigned char> ConvertTo8U(const cv::Mat &src);

/**
 * @brief Cumulative sum
 * @param src matrix
 * @param dst matrix
 */
void CumSum(const cv::Mat1f &src, cv::Mat1f &dst);

/**
 * @brief Get string representation of matrix type
 * @param matrix
 * @return type as string
 */
std::string MatTypeToString(const cv::Mat& m);

} // sem namespace

#endif // SEM_CORE_MAT_UTILS_H_

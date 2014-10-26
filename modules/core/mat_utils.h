#ifndef SEM_CORE_MAT_UTILS_H_
#define SEM_CORE_MAT_UTILS_H_

#include <string>
#include "core/typedefs.h"
#include "core/exception.h"

#include <iostream>
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


/**
 * @brief Create Mat object (row vector) and fill with range
 * @param start value
 * @param stop value (exclusive)
 * @param step value
 * @return template
 */
template <class T>
cv::Mat_<T> ARange_(T start, T stop, T step)
{
    double diff = static_cast<double>(stop)-static_cast<double>(start);

    if((diff > 0 && step < 0) || (diff < 0 && step > 0)) {

        SEM_THROW_VALUE_ERROR("step incompatible with start and stop.");
    }

    int cols;
    if(step == 0) {

        cols = 0;
    }
    else {

        // Need full integer columns
        double tmp = (diff < 0)? abs(diff) : diff;
        tmp = (tmp+1)/fabs(static_cast<double>(step));
        cols = static_cast<int>(tmp);
    }

    cv::Mat_<T> m(1, cols);
    T value = start;
    int count = 0;
    bool done = (step > 0)? value >= stop : value <= stop;
    while(!done && count < cols) {

        m(count++) = value;
        value += step;
        done = (step > 0)? value >= stop : value <= stop;
    }

    // exclude stop value
    if(count < cols) { m = m.colRange(0, count); }

    return m;
}

} // sem namespace

#endif // SEM_CORE_MAT_UTILS_H_

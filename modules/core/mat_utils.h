#ifndef SEM_CORE_MAT_UTILS_H_
#define SEM_CORE_MAT_UTILS_H_

#include <string>
#include "core/typedefs.h"
#include "core/exception.h"

#include <iostream>
namespace sem {

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
cv::Mat_<T> ARange(T start, T stop, T step)
{
    int cols = static_cast<int>(stop)-static_cast<int>(start);

    std::cout<<"c"<<cols<<std::endl;

    if((cols > 0 && step < 0) || (cols < 0 && step > 0)) {

        SEM_THROW_VALUE_ERROR("step incompatible with start and stop.");
    }
    else if(cols < 0) { cols = abs(cols); }

    if(step == 0) {

        cols = 0;
    }
    else {

        // Need full integer columns
        float tmp = cols/static_cast<float>(step);
        std::cout<<"t "<<tmp<<std::endl;
        cols = static_cast<int>(tmp);
        if(cols > 0 && cols == tmp) cols--; // exclude stop
    }

    cv::Mat_<T> m(1, cols);
    T value = start;
    for(int i=0; i<cols; i++) {

        m(i) = value;
        value += step;
    }

    return m;
}

} // sem namespace

#endif // SEM_CORE_MAT_UTILS_H_

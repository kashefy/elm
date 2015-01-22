/** @file template utility definitions that have to be defined inline
  */
#ifndef ELM_CORE_MAT_UTILS_INL_H__
#define ELM_CORE_MAT_UTILS_INL_H__

#include "elm/core/cv/typedefs_fwd.h"
#include "elm/core/defs.h"
#include "elm/core/exception.h"

namespace elm {

/**
 * @brief get first index of element with a specific value in matrix
 * Inspired by this Stack Overflow post: @see http://stackoverflow.com/questions/25835587/find-element-in-opencv-mat-efficiently
 *
 * @param[in] matrix to search in
 * @param[in] value to search for
 * @param[out] first index containing this value, only meaningful if found
 * @return true if sucessfully found, always flase for empty input.
 *
 * @throws ExceptionBadDims if no. of channels != 1, ExceptionTypeError for non-continuous matrix input.
 */
template <class T>
bool find_first_of(const cv::Mat &m, const T &value, int &index=sem::NA_IDX)
{
    if(!m.empty()) {

        if(m.channels() != 1) { ELM_THROW_BAD_DIMS("Only single-channel matrices supported for now."); }
        if(!m.isContinuous()) { ELM_THROW_TYPE_ERROR("Only continuous matrices supported for now."); }
    }
    else if(m.channels() > 1) { ELM_THROW_BAD_DIMS("Only single-channel matrices supported for now."); }

    for(int r=0; r < m.rows; r++) {

        const T* row = m.ptr<T>(r);
        const T* result = std::find(row, row + m.cols, value);
        if(result != row + m.cols) {

            index = static_cast<int>(result - m.ptr<T>(0));
            return true;
        }
    }
    index = -1;
    return false;
}

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

        ELM_THROW_VALUE_ERROR("step incompatible with start and stop.");
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

} // elm namespace

#endif // ELM_CORE_MAT_UTILS_INL_H__

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
  * @brief Convert 2d point of integers to single channel Mat of integers
  * @param 2d point of integers
  * @return single channel row-matrix of integers [x, y]
  */
cv::Mat1i Point2Mat(const cv::Point2i& p);

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
  * @param[in] border type - \see OpenCV's copyMakeBorder() or borderInterpolate() for details.
  * @param[in] value - Border value if border type==BORDER_CONSTANT
  */
void NeighMeanVar(const cv::Mat1f& m, int radius, cv::Mat1f &neigh_mean, cv::Mat1f &neigh_var,
                  int border_type=cv::BORDER_REPLICATE, const cv::Scalar &value=cv::Scalar());

/**
 * @brief Get all element values at a position across different matrices
 * @param vector of matrices
 * @param row
 * @param col
 * @return row matrix with extracted elements
 * @throws ExceptionBadDims for positions that cannot be accessed.
 * @todo validate equally sized matrices inside vector or define protocol
 * current behavior, rely on dims of first vector entry
 */
cv::Mat1f ElementsAt(const VecMat1f &v, int row, int col);

/**
 * @brief Reshape vector of mat to single mat with row per element and col per vector element/block/kernel.
 * Only applicable to vector of equally sized matrices.
 * @param input vector of matrices
 * @return single matrix, row per matrix element and cols=vector size, all dims are non-zero
 * @todo enforce validation of same-dim matrix elements or define clear protocol, current behavior: rely on dims of first vector entry
 */
cv::Mat1f Reshape(const VecMat1f &v);

/**
 * @brief Function for converting a Mat_ object into a vector of same type
 * The mat is flattened beforehand.
 * Involves copying elements
 * May only work with matrices of POD (e.g. int, float, uchar,...)
 * @param matrix
 * @return resulting vector with flattened matrix data
 */
template <typename T>
std::vector<T> Mat_ToVec_(const cv::Mat_<T> &m) {

    // syntax learned from posts here:
    // http://stackoverflow.com/questions/610245/where-and-why-do-i-have-to-put-the-template-and-typename-keywords
    const T* p = m.template ptr<T>(0);
    std::vector<T> v(p, p+m.total());
    return v;
}

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
static int _NA=-1;   ///< not applicable
template <class T>
bool find_first_of(const cv::Mat &m, const T &value, int &index=_NA)
{
    if(!m.empty()) {

        if(m.channels() != 1) { SEM_THROW_BAD_DIMS("Only single-channel matrices supported for now."); }
        if(!m.isContinuous()) { SEM_THROW_TYPE_ERROR("Only continuous matrices supported for now."); }
    }
    else if(m.channels() > 1) { SEM_THROW_BAD_DIMS("Only single-channel matrices supported for now."); }

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

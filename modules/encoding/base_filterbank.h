/** @file Define classes for encapsulating, applying filter banks
 * and different response iterators (per kernel, per element across kernels).
 */
#ifndef SEM_ENCODING_BASE_FILTERBANK_H_
#define SEM_ENCODING_BASE_FILTERBANK_H_

#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "core/exception.h"
#include "core/typedefs.h"

/**
 * @brief base class for filter banks
 *  The interface should aid in iterating through response
 *  by defining different iterators,
 *  whether kernel-by-kernel or element response acorss all kernels
 */
class base_FilterBank
{
public:
    /** Empty default destructor
     */
    virtual ~base_FilterBank();

    /**
     * @brief Convolve stimulus with each kernel
     * @param stimulus
     * @return response per kernel
     */
    virtual VecMat1f Compute(cv::Mat1f stimulus);

    /**
     * @brief Get element response across all kernels
     * @param row
     * @param column
     * @return element response row matrix no. of cols==filter bank fan-out (e.g. no. of kernels)
     */
    virtual cv::Mat1f ElementResponse(int row, int col);

protected:
    /**
     * @brief Empty default constructor, only accessible by child classes
     */
    base_FilterBank();

    VecMat1f kernels_;  ///< individual kernels
    VecMat1f response_; ///< response per kernel for most recent input
};

#endif // SEM_ENCODING_BASE_FILTERBANK_H_

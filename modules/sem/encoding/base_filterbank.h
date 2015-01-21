/** @file Define classes for encapsulating, applying filter banks
 * and different response iterators (per kernel, per element across kernels).
 */
#ifndef SEM_ENCODING_BASE_FILTERBANK_H_
#define SEM_ENCODING_BASE_FILTERBANK_H_

#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "sem/core/exception.h"
#include "sem/core/typedefs_sfwd.h"

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
    virtual sem::VecMat1f Compute(cv::Mat1f stimulus);

    /**
     * @brief Get element response across all kernels
     * @param row
     * @param column
     * @return element response row matrix no. of cols==filter bank fan-out (e.g. no. of kernels)
     */
    virtual cv::Mat1f ElementResponse(int r, int c) const;

    /**
     * @brief Get underlying kernels
     * Mostly meant for testing purposes because it can involve a heavy copy of matrices.
     * @return vector of kernels
     */
    virtual sem::VecMat1f Kernels() const = 0;

    /**
     * @brief Get response to most recent stimulus
     * May involve heavy copy operations.
     * @return vector of response matrices
     */
    virtual sem::VecMat1f Response() const;

    /**
     * @brief size of the filter bank (e.g. no. of kernels)
     * @return size of the filter bank
     */
    virtual size_t size() const;

protected:
    /**
     * @brief Empty default constructor, only accessible by child classes
     */
    base_FilterBank();

    /**
     * @brief Called by Compute for rectifying response from individual kernels
     * (e.g. square response). Default is to do nothing and leave response as is.
     * @param response modified in-place
     */
    virtual void Rectify(cv::Mat1f &response);

    sem::VecMat1f kernels_;  ///< individual kernels
    sem::VecMat1f response_; ///< response per kernel for most recent input
};

#endif // SEM_ENCODING_BASE_FILTERBANK_H_

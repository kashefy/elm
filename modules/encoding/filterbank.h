#ifndef SEM_ENCODING_BASE_FILTERBANK_H_
#define SEM_ENCODING_BASE_FILTERBANK_H_

#include <opencv2/core.hpp>

#include "core/typedefs.h"

/**
 * @brief base class for filter banks
 *  The interface should aid in iterating through response
 *  whether kernel-by-kernel or element response acorss all kernels
 */
class base_FilterBank
{
public:
    ~base_FilterBank();

    /**
     * @brief Compute response to all kernels
     * @param stimulus
     * @return response per kernel
     */
    VecMat1f Compute(cv::Mat1f stimulus) = 0;


protected:
    base_FilterBank();

    VecMat1f kernels_;  ///< individual kernels
    VecMat1f response_; ///< response per kernel for most recent input
};

#endif // SEM_ENCODING_BASE_FILTERBANK_H_

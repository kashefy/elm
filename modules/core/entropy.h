#ifndef SEM_CORE_ENTROPY_H_
#define SEM_CORE_ENTROPY_H_

#include <opencv2/core.hpp>

namespace sem {

/**
 * @brief Compute the normalized conditional entropy of a probability distribution
 * @param pdf
 * @return normalized cond. entropy
 */
float CondEntropy(const cv::Mat1f &pdf);

} // namespace sem

#endif // SEM_CORE_ENTROPY_H_

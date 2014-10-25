#ifndef SEM_CORE_ENTROPY_H_
#define SEM_CORE_ENTROPY_H_

#include <opencv2/core.hpp>

namespace sem {

/**
 * @brief Compute the normalized conditional entropy of a probability distribution
 *
 * @param pdf as matrix of vector of floats
 * @return normalized cond. entropy. Returns 0 on empty input.
 */
float CondEntropy(cv::InputArray &pdf);

} // namespace sem

#endif // SEM_CORE_ENTROPY_H_

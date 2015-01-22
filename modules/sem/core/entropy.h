#ifndef ELM_CORE_ENTROPY_H_
#define ELM_CORE_ENTROPY_H_

#include <opencv2/core.hpp>

namespace elm {

/**
 * @brief Compute the normalized conditional entropy of a probability distribution
 *
 * @param pdf as matrix of vector of floats
 * @return normalized cond. entropy. Returns 0 on empty input.
 */
float CondEntropy(cv::InputArray &pdf);

} // namespace elm

#endif // ELM_CORE_ENTROPY_H_

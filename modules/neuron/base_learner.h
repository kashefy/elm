#ifndef SEM_NEURON_BASE_LEARNER_H_
#define SEM_NEURON_BASE_LEARNER_H_

#include "core/typedefs.h"

/**
 * @brief Define interface for learning neurons
 */
class base_Learner
{
public:
    virtual ~base_Learner() {}

    /**
     * @brief learn from most provided state and/or prediction
     * @param target state
     */
    virtual void Learn(const cv::Mat &target) = 0;

    /**
     * @brief predict learner state
     * @param evidence
     * @return learner state given latest evidence
     */
    virtual cv::Mat Predict(const cv::Mat& evidence) = 0;

    /**
     * @brief Get learner's state
     * @return learner's state from most recent prediction
     */
    virtual cv::Mat State() const = 0;

protected:
    base_Learner();
};

#endif // SEM_NEURON_BASE_LEARNER_H_

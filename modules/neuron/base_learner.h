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
     * @brief learn from most recent state and/or prediction
     */
    virtual void Learn() = 0;

    /**
     * @brief predict learner state
     * @param evidence
     * @return learner state given latest evidence
     */
    virtual cv::Mat Predict(const cv::Mat& evidence) = 0;

protected:
    base_Learner();
};

#endif // SEM_NEURON_BASE_LEARNER_H_

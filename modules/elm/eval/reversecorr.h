/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, Elm Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_EVAL_REVERSECORR_H_
#define _ELM_EVAL_REVERSECORR_H_

#include <opencv2/core.hpp>

/**
 * @brief base class for Reverse correlation algorithms
 */
class base_ReverseCorr
{
public:
    virtual ~base_ReverseCorr();

    /**
     * @brief Add evidence (e.g. vector of spikes)
     * @param in
     */
    virtual void Add(cv::InputArray in) = 0;

    /**
     * @brief Compute reverse correlation
     * @return matrix representing reverse correlation
     */
    virtual cv::Mat1f Compute() const = 0;

protected:
    base_ReverseCorr();
};

/**
 * @brief class for computing spike-triggered average
 */
class STA : public base_ReverseCorr
{
public:
    STA();

    /**
     * @brief Add spikes
     * @param in
     * @throws if dimensions of spike matrix/vector change
     */
    void Add(cv::InputArray in);

    cv::Mat1f Compute() const;

private:
    cv::Mat1f sta_;     ///< running spike-triggered average
    int nb_samples_;    ///< no. of accumulated evidence
};

#endif // _ELM_EVAL_REVERSECORR_H_

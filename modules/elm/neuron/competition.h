/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_NEURON_COMPETITION_H_
#define _ELM_NEURON_COMPETITION_H_

#include "elm/core/typedefs_sfwd.h"

namespace elm {

class base_Learner;

/**
 * @brief Base class for defining competition between learners
 * suitable for unsupervised learning
 */
class base_Competition
{
public:
    virtual ~base_Competition();

    /**
     * @brief Let learners compete
     * @param reference to vector of learners, manipulate in place
     * @return competition outcome
     */
    virtual cv::Mat Compete(std::vector<std::shared_ptr<base_Learner> >& learners) = 0;

protected:
    base_Competition();
};

/**
 * @brief Define interfaces for a Winner-Take-All circuit for spiking neurons
 */
class base_WTA : public base_Competition
{
public:
    virtual ~base_WTA();

    /**
     * \see base_Competition::Compete
     */
    virtual cv::Mat Compete(std::vector<std::shared_ptr<base_Learner> > &learners) = 0;

protected:
    /**
     * @brief base_WTA constructor
     * @param delta_t_msec time resolution in milliseconds
     */
    base_WTA(float delta_t_msec);

    float delta_t_sec_;    ///< time resolution in seconds
};

} // namespace elm

#endif // _ELM_NEURON_COMPETITION_H_

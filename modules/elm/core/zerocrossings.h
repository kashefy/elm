/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, Elm Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_CORE_ZEROCROSSINGS_H_
#define _ELM_CORE_ZEROCROSSINGS_H_

#include "elm/core/cv/typedefs_fwd.h"

/**
 * @brief base class for computing zero corssings
 */
class base_ZeroCrossings
{
public:
    virtual ~base_ZeroCrossings();

    virtual void operator ()(const cv::Mat1f &src, cv::Mat1f &dst) const = 0;

protected:
    base_ZeroCrossings();
};

/**
 * @brief Use forward difference (1st derivative) to detect zero crossings
 * src[i]*src[i+1] < 0 indicates a zero crossing.
 * Perform on both x and y axes.
 */
class ZeroCrossingsDiff : public base_ZeroCrossings
{
public:
    ZeroCrossingsDiff();

    /**
     * @brief Calculate forward difference for detecting zero crossings
     * @param[in] input of floats
     * @param[out] dst
     */
    void operator ()(const cv::Mat1f &src, cv::Mat1f &dst) const;
};

#endif // _ELM_CORE_ZEROCROSSINGS_H_

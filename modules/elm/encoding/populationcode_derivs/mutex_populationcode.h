/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_ENCODING_MUTEX_POPULATIONCODE_H_
#define _ELM_ENCODING_MUTEX_POPULATIONCODE_H_

#include "elm/encoding/base_populationcode.h"

namespace elm {

/**
 * @brief Mutually exclusive population code (a.k.a simple pop. code)
 * Fan out is 2 (i.e. each stimulus element is expanded to 2 pop. code elements)
 */
class MutexPopulationCode : public base_PopulationCode
{
public:
    MutexPopulationCode();

    MutexPopulationCode(const LayerConfig &config);

    virtual void State(const cv::Mat1f& in);

    /**
     * @brief PopCode
     * @return pop code with double the input columns
     */
    virtual cv::Mat1f PopCode();
};

} // namespace elm

#endif // _ELM_ENCODING_MUTEX_POPULATIONCODE_H_

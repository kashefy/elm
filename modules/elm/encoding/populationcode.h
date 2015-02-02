/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, Elm Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_ENCODING_POPULATIONCODE_H_
#define _ELM_ENCODING_POPULATIONCODE_H_

#include <memory>

#include <boost/optional.hpp>

#include <opencv2/core.hpp>

#include "elm/core/base_Layer.h"
#include "elm/core/layerconfig.h"
#include "elm/core/typedefs_sfwd.h"

namespace elm {

class base_FilterBank;

/**
 * @brief The base class for population codes
 */
class base_PopulationCode : public base_Layer
{
public:
    static const std::string KEY_INPUT_STIMULUS;
    static const std::string KEY_OUTPUT_POP_CODE;

    virtual ~base_PopulationCode();

    /**
     * @brief Compute internal state
     * @param in input
     * @param kernels vector of kernels
     *
     * @todo Drop kernel parameter, when filter bank becomes a layer
     */
    virtual void State(const cv::Mat1f& in, const elm::VecMat1f& kernels=elm::VecMat1f()) = 0;

    /**
     * @brief get population code
     * @return population code
     */
    virtual cv::Mat1f PopCode() = 0;

    /* Layer derived methods:
     * ---------------------
     */
    virtual void Clear();

    virtual void Reconfigure(const LayerConfig &config);

    virtual void Reset(const LayerConfig &config);

    virtual void IONames(const LayerIONames &config);

    /**
     * @brief compute state and population code
     * effectivey a call of State() and PopCode() methods
     */
    virtual void Activate(const Signal &signal);

    /**
     * @brief Appends pop code, and state if requested
     * @param signal populated with response
     */
    virtual void Response(Signal &signal);

protected:
    base_PopulationCode();

    base_PopulationCode(const LayerConfig &config);

    std::string name_stimulus_; ///< name of stimulus in signal object
    std::string name_pop_code_; ///< destination name of popuation code in signal

    cv::Mat1f pop_code_;    ///< most recent population code
};

/**
 * @brief Mutually exclusive population code (a.k.a simple pop. code)
 */
class MutexPopulationCode : public base_PopulationCode
{
public:
    MutexPopulationCode();

    MutexPopulationCode(const LayerConfig &config);

    virtual void State(const cv::Mat1f& in, const elm::VecMat1f& kernels=elm::VecMat1f());

    virtual cv::Mat1f PopCode();
};

/**
 * @brief a base class for population codes that are stateful
 */
class base_StatefulPopulationCode : public base_PopulationCode
{
public:
    static const std::string KEY_OUTPUT_OPT_STATE;  ///< optional output for state due to most recent stimuli

protected:
    virtual void Clear();

    virtual void IONames(const LayerIONames &config);

    virtual void Response(Signal &signal);

    base_StatefulPopulationCode();

    base_StatefulPopulationCode(const LayerConfig &config);

    // members
    OptS name_state_;   ///< optional destination name for state in signal object

    cv::Mat1f state_;   ///< internal state due to most recent stimuli
};

/**
 * @brief Population code sampled from distribution
 * (e.g. soft-max oriented gabor filter response)
 */
class SoftMaxPopulationCode : public base_PopulationCode
{
public:
    SoftMaxPopulationCode();

    /**
     * @brief Compute distribution for each node from different kernel responses
     * @param input stimulus
     * @param kernels (e.g. filter bank)
     */
    virtual void State(const cv::Mat1f& in, const elm::VecMat1f& kernels=elm::VecMat1f());

    /**
     * @brief Compute distribution for each node from different kernel responses
     * Use predefined kernels inside a filter bank
     * @param filter_bank, ownership kept by caller
     */
    virtual void State(const cv::Mat1f &in, std::unique_ptr<base_FilterBank> const &filter_bank);

    /**
     * @brief Determine population code per input node by sampling from response distribution
     * @return population code for all input nodes
     */
    virtual cv::Mat1f PopCode();

protected:
    /**
     * @brief Normalize kernel response by global factor
     * @param response normalized in-place
     */
    virtual void Normalize(elm::VecMat1f &response) const;

    VecMat1f state_;            ///< internal state
    VecMat1f response_distr_;   ///< response distribution
    int fan_out_;               ///< dimensions of state per node (e.g. no. of kernels)

};

} // namespace elm

#endif // _ELM_ENCODING_POPULATIONCODE_H_

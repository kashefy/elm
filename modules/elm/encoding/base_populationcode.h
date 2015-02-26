/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_ENCODING_BASE_POPULATIONCODE_H_
#define _ELM_ENCODING_BASE_POPULATIONCODE_H_

#include <opencv2/core/core.hpp>

#include "elm/core/base_Layer.h"
#include "elm/core/typedefs_sfwd.h"

namespace elm {

namespace detail {

const std::string BASE_POPULATIONCODE__KEY_INPUT_STIMULUS   = "input";    ///< define string here to ensure early initialization for test purposes
const std::string BASE_POPULATIONCODE__KEY_OUTPUT_POP_CODE = "pop_code";      ///< define string here to ensure early initialization for test purposes

}

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
     * @param in input, see expected dimension required by dervied class
     *
     * @todo Drop kernel parameter, when filter bank becomes a layer
     */
    virtual void State(const cv::Mat1f& in) = 0;

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

    virtual void InputNames(const LayerInputNames &config);

    virtual void OutputNames(const LayerOutputNames &config);

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

} // namespace elm

#endif // _ELM_ENCODING_BASE_POPULATIONCODE_H_

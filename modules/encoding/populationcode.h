#ifndef SEM_ENCODING_POPULATIONCODE_H_
#define SEM_ENCODING_POPULATIONCODE_H_

#include <memory>

#include "core/base_Layer.h"
#include "core/layerconfig.h"
#include "core/typedefs.h"

class base_FilterBank;

/**
 * @brief The base class for population codes
 */
class base_PopulationCode : public base_Layer
{
public:
    static const std::string PARAM_KERNELS;

    static const std::string KEY_INPUT_STIMULUS;
    static const std::string KEY_OUTPUT_STATE;
    static const std::string KEY_OUTPUT_POP_CODE;

    virtual ~base_PopulationCode();

    /**
     * @brief Compute internal state
     * @param in input
     * @param kernels vector of kernels
     */
    virtual void State(const cv::Mat1f& in, const VecMat1f& kernels=VecMat1f()) = 0;

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

    virtual void Stimulus(const Signal &signal);

    /**
     * @brief compute state and population codeApply
     */
    virtual void Apply();

    /**
     * @brief Appends pop code, and state if requested
     * @param signal populated with response
     */
    virtual void Response(Signal &signal);

protected:
    base_PopulationCode();

    std::string name_stimulus_;
    OptS name_state_;
    std::string name_pop_code_;

    cv::Mat1f stimulus_;
    cv::Mat state_;
    cv::Mat1f pop_code_;

    VecMat1f kernels_;
};

/**
 * @brief Mutually exclusive population code (a.k.a simple pop. code)
 */
class MutexPopulationCode : public base_PopulationCode
{
public:
    MutexPopulationCode();

    virtual void State(const cv::Mat1f& in, const VecMat1f& kernels=VecMat1f());

    virtual cv::Mat1f PopCode();

protected:
    cv::Mat1f state_;    ///< internal state
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
    virtual void State(const cv::Mat1f& in, const VecMat1f& kernels=VecMat1f());

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
    virtual void Normalize(VecMat1f &response) const;

    VecMat1f state_;            ///< internal state
    VecMat1f response_distr_;   ///< response distribution
    int fan_out_;               ///< dimensions of state per node (e.g. no. of kernels)

};

#endif // SEM_ENCODING_POPULATIONCODE_H_

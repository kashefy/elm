#ifndef _ELM_CORE_BASE_LAYER_DERIVS_BASE_MATOUTPUTLAYER_H
#define _ELM_CORE_BASE_LAYER_DERIVS_BASE_MATOUTPUTLAYER_H

#include <string>

#include <opencv2/core.hpp>

#include "elm/core/base_Layer.h"

namespace elm {

class Signal;

namespace detail {

const std::string BASE_MATOUTPUT_LAYER__KEY_OUTPUT_RESPONSE = "m";   ///< define string here to ensure early initialization for test purposes

}

/**
 * @brief class for time-invariant layer
 * @todo generalize to FeatureOutputLayer, or maybe keep this (compiles faster)
 */
class base_MatOutputLayer : virtual public base_Layer
{
public:
    static const std::string KEY_OUTPUT_RESPONSE;

    virtual ~base_MatOutputLayer();

    virtual void IONames(const LayerIONames &io);

    virtual void Response(Signal &signal);

protected:
    base_MatOutputLayer();

    base_MatOutputLayer(const LayerConfig& cfg);

    // members
    std::string name_output_;    ///< destination name in signal object

    cv::Mat1f m_;   ///< most recent response/output
};

} // namespace elm

#endif // _ELM_CORE_BASE_LAYER_DERIVS_BASE_MATOUTPUTLAYER_H

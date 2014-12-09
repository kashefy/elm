#ifndef SEM_CORE_BASE_LAYER_H_
#define SEM_CORE_BASE_LAYER_H_

#include <opencv2/core.hpp>

#include "core/layerconfig.h"

class Signal;

/**
 * @brief The Layer base class
 */
class base_Layer
{
public:
    virtual ~base_Layer();

    /**
     * @brief Reset the state of the layer.
     */
    virtual void Reset() = 0;

    /**
     * @brief Reset the state of the layer with new configurations.
     * This is a hard reset.
     * For a soft reset \see Reconfigure
     * @param config
     */
    virtual void Reset(const LayerConfig& config);

    /**
      * @brief Reconfigure the layer without modifying its state
      * \todo Keep this?
      * @param new configuration
      */
    virtual void Reconfigure(const LayerConfig& config) = 0;

    /**
     * @brief Set stimulus
     */
    virtual void Stimulus(const Signal &signal) = 0;

    /**
     * @brief Apply layer computations to most recent stimuli
     */
    virtual void Apply() = 0;

    /**
     * @brief Get layer response
     */
    virtual void Response(Signal &signal) = 0;

protected:
    base_Layer();

    /**
     * @brief Construct a layer with given configurations
     * @param config
     */
    base_Layer(const LayerConfig& config);
};

#endif // SEM_CORE_BASE_LAYER_H_

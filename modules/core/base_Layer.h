#ifndef SEM_CORE_BASE_LAYER_H_
#define SEM_CORE_BASE_LAYER_H_

#include <opencv2/core.hpp>

#include "core/layerconfig.h"

/**
 * @brief The Layer base class
 */
class base_Layer
{
public:
    virtual ~base_Layer();

    /**
     * @brief Reset the state of the layer
     */
    virtual void Reset() = 0;

    /**
     * @brief Reset the state of the layer with new configurations
     * @param config
     */
    virtual void Reset(const LayerConfig& config);

    /**
     * @brief Set stimulus
     */
    virtual void Stimulus() = 0;

    /**
     * @brief Apply layer computations
     */
    virtual void Apply() = 0;

    /**
     * @brief Get layer response
     */
    virtual void Response() = 0;

protected:
    base_Layer();

    /**
     * @brief Construct a lyer with given configurations
     * @param config
     */
    base_Layer(const LayerConfig& config);
};

#endif // SEM_CORE_BASE_LAYER_H_

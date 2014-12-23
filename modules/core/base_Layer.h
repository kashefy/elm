#ifndef SEM_CORE_BASE_LAYER_H_
#define SEM_CORE_BASE_LAYER_H_

#include <opencv2/core.hpp>

#include "core/layerconfig.h"

class Signal;

/**
 * @brief The Layer base class
 * Overloaded constructor calls overloaded Reset() so you can implement Reset(config) directly.
 */
class base_Layer
{
public:
    virtual ~base_Layer();

    /**
     * @brief Reset the state of the layer.
     */
    virtual void Clear() = 0;

    /**
     * @brief Reset the state of the layer with new configurations.
     * This is a hard reset.
     * For a soft reset \see Reconfigure
     * @param config
     */
    virtual void Reset(const LayerConfig& config);

    /**
      * @brief Reconfigure the layer without modifying its state
      * @todo Keep this?
      * @param new configuration
      */
    virtual void Reconfigure(const LayerConfig& config) = 0;

    /**
      * @brief Set layer input output keys
      * @param new I/O configuration
      */
    virtual void IO(const LayerIO& config) = 0;

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
     * @brief Construct a layer with given configurations, calls Reset(config)
     * @param config
     */
    base_Layer(const LayerConfig& config);
};

/**
 * @brief class for defining interfaces of an abstract layer that can learn
 * Overloaded constructor calls overloaded Reset() so you can implement Reset(config) directly.
 */
class base_LearningLayer : public base_Layer
{
public:
    virtual ~base_LearningLayer();

//    /**
//     * @brief Reset the state of the layer.
//     */
//    virtual void Clear() = 0;

//    /**
//      * @brief Reconfigure the layer without modifying its state
//      * \todo Keep this?
//      * @param new configuration
//      */
//    virtual void Reconfigure(const LayerConfig& config) = 0;

//    /**
//     * @brief Set stimulus
//     */
//    virtual void Stimulus(const Signal &signal) = 0;

//    /**
//     * @brief Apply layer computations to most recent stimuli
//     */
//    virtual void Apply() = 0;

//    /**
//     * @brief Get layer response
//     */
//    virtual void Response(Signal &signal) = 0;

        /**
         * @brief Learn from most recent stimuli
         */
        virtual void Learn() = 0;

protected:
    base_LearningLayer();

    /**
     * @brief Construct a layer with given configurations, calls Reset(config)
     * @param config
     */
    base_LearningLayer(const LayerConfig& config);
};

#endif // SEM_CORE_BASE_LAYER_H_

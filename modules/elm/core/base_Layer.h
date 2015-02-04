/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, Elm Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_CORE_BASE_LAYER_H_
#define _ELM_CORE_BASE_LAYER_H_

namespace elm {

class LayerConfig;
class LayerIONames;
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
      * @brief Set layer input and output names
      *
      * Calls InputNames() and OutputNames() methods implemented by derived classes.
      *
      * @param new I/O configuration
      */
    void IONames(const LayerIONames& io);

    /**
      * @brief Set layer input names
      * @param new I/O configuration
      */
    virtual void InputNames(const LayerIONames& io) = 0;

    /**
      * @brief Set layer output names
      * @param new I/O configuration
      */
    virtual void OutputNames(const LayerIONames& io) = 0;

    /**
     * @brief Activate layer computations to most recent stimuli
     * This is expected to perform the heavy lifting
     *
     * @param[in] signal object with input features
     * @throws elm::ExceptionKeyError for missing features
     */
    virtual void Activate(const Signal &signal) = 0;

    /**
     * @brief Get layer response due to most recent stimuli
     *
     * @param[out] signal object to populate with layer's output features
     */
    virtual void Response(Signal &signal) = 0;

protected:
    base_Layer();

    /**
     * @brief Construct and fully configure a layer (parameters and IO)
     * Derived class shoudl call Reset(config) and IO(config) from here.
     * @param config
     */
    base_Layer(const LayerConfig& config);
};

} // namespace elm

#endif // _ELM_CORE_BASE_LAYER_H_

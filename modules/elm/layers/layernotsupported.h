/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_LAYERS_LAYERNOTSUPPORTED_H_
#define _ELM_LAYERS_LAYERNOTSUPPORTED_H_

#include <string>

#include "elm/core/base_Layer.h"

namespace elm {

/**
 * @brief This is not an ordinary layer implementation
 * This derived layer only aids in conditionally defined layers
 * (i.e layers that are only supported if specific dependencies exist)
 *
 * @throws elm::ExceptionNotImpl for any calls to its methods
 */
class base_LayerNotSupported : public base_Layer
{
public:
    /** @throws elm::ExceptionNotImpl when called
     */
    void Clear();

    /** @throws elm::ExceptionNotImpl when called
     * Arguments are completely ignored.
     */
    void Reset(const LayerConfig& config);

    /** @throws elm::ExceptionNotImpl when called
     * Arguments are completely ignored.
     */
    void Reconfigure(const LayerConfig& config);

    /** @throws elm::ExceptionNotImpl when called
     * Arguments are completely ignored.
     */
    void InputNames(const LayerIONames &io);

    /** @throws elm::ExceptionNotImpl when called
     * Arguments are completely ignored.
     */
    void OutputNames(const LayerIONames &io);

    /** @throws elm::ExceptionNotImpl when called
     * Arguments are completely ignored.
     */
    void Activate(const Signal &signal);

    /** @throws elm::ExceptionNotImpl when called
     * Arguments are completely ignored.
     */
    void Response(Signal &signal);

protected:
    base_LayerNotSupported(const std::string &message);

    base_LayerNotSupported();

    base_LayerNotSupported(const LayerConfig &config, const std::string msg=std::string());

    /**
     * @brief throw the elm::ExceptionNotImpl with a message.
     * Called by all methods.
     */
    void ThrowException() const;

    // members
    std::string msg_;   ///< message passed to exception when thrown
};

/** Macros to quickly implement a layer as not implemented
 */
#define ELM_LAYER_NOT_SUPPORTED(TDerived, msg) base_LayerNotSupported  {        \
    public:                                                                     \
    TDerived(const std::string message=msg) : base_LayerNotSupported(msg) {}    \
    TDerived(const LayerConfig &config, const std::string message=msg) : base_LayerNotSupported(config, msg) {} \
                                                                       }

} // namespace elm

#endif // _ELM_LAYERS_LAYERNOTSUPPORTED_H_

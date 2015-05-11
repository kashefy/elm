/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_LAYERS_LAYERFACTORY_H_
#define _ELM_LAYERS_LAYERFACTORY_H_

#include <memory>
#include <string>

#include "elm/core/typedefs_fwd.h"

namespace elm {

/**
 * @brief class for implementing layer related factory methods
 * Such as instantiation and sequencing of multiple layer applications (e.g. pipeline)
 * @todo enable post hoc addition of custom layer instances
 */
class LayerFactory
{
public:
    typedef std::string LayerType;

    LayerFactory();

    /**
     * @brief Create smart pointer to an instantiated layer
     * @param type
     * @return pointer to layer instance
     * @throws ExceptionTypeError on unrecognized layer type
     */
    static std::shared_ptr<base_Layer> CreateShared(const LayerType &type);

    /**
     * @brief Create smart pointer to an instantiated layer
     * @param type
     * @param configuration
     * @param I/O names
     * @return pointer to layer instance
     * @throws ExceptionTypeError on unrecognized layer type
     */
    static std::shared_ptr<base_Layer> CreateShared(const LayerType &type,
                                                    const LayerConfig &config,
                                                    const LayerIONames &io);

    /**
     * @brief Fuly initialize layer instance
     * @param layer object reference
     * @param config paramters
     * @param io I/O names
     * @throws ExceptionValueError for unintialized pointers
     */
    static void Init(std::shared_ptr<base_Layer> &layer,
                     const LayerConfig &config,
                     const LayerIONames &io);
};

} // namesapce elm

#endif // _ELM_LAYERS_LAYERFACTORY_H_

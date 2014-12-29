#ifndef SEM_LAYERFACTORY_H_
#define SEM_LAYERFACTORY_H_

#include <memory>
#include <string>

#include "core/base_Layer.h"
#include "core/exception.h"

/**
 * @brief class for implementing layer related factory methods
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
    static std::shared_ptr<base_Layer> CreateLayerPtrShared(const LayerType &type);

};

#endif // SEM_LAYERFACTORY_H_

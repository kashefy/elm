#include "layers/layerfactory.h"

#include <boost/assign/list_of.hpp>

#include "core/registor.h"

/** Whenever a new layer is imeplemented:
 *  1. include its header below
 *  2. Add it to the initialization of g_layerRegistry map.
 */
#include "layers/layer_z.h"
#include "encoding/populationcode.h"
#include "layers/saliencyitti.h"
#include "layers/weightedsum.h"

using boost::assign::map_list_of;

typedef Registor_<base_Layer> LayerRegistor;
typedef Registor_<base_Layer>::Registry LayerRegistry;

/** Macros for adding individual instances to registry
 *  credit: J. Turcot, T. Senechal, http://stackoverflow.com/questions/138600/initializing-a-static-stdmapint-int-in-c
 */
#define ADD_TO_REGISTRY(Registor, NewInstance) (#NewInstance, &Registor::DerivedInstance<NewInstance>)
#define ADD_TO_LAYER_REGISTRY(NewInstance) ADD_TO_REGISTRY(LayerRegistor, NewInstance)

LayerRegistry g_layerRegistry = map_list_of
        ADD_TO_LAYER_REGISTRY( LayerZ )
        ADD_TO_LAYER_REGISTRY( MutexPopulationCode )
        ADD_TO_LAYER_REGISTRY( SaliencyItti )
        ADD_TO_LAYER_REGISTRY( WeightedSum )
        ; ///< <-- add new layer to registry here

LayerFactory::LayerFactory()
{
}

LayerRegistor::RegisteredTypeSharedPtr LayerFactory::CreateLayerPtrShared(const LayerType &type)
{
    return LayerRegistor::CreatePtrShared(g_layerRegistry, type);
}

LayerRegistor::RegisteredTypeSharedPtr LayerFactory::CreateLayerPtrShared(const LayerType &type,
                                                                          const LayerConfig &config,
                                                                          const LayerIONames &io)
{
    LayerRegistor::RegisteredTypeSharedPtr ptr = LayerFactory::CreateLayerPtrShared(type);
    ptr->Reset(config);
    ptr->IONames(io);
    return ptr;
}

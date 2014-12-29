#include "layers/layerfactory.h"

#include <boost/assign/list_of.hpp>

#include "core/registor.h"

/** Whenever a new layer is imeplemented:
 *  1. include its header below
 *  2. Add it to the initialization of g_layerRegistry map.
 */
#include "layers/layer_z.h"
#include "layers/saliencyitti.h"
#include "layers/weightedsum.h"

using boost::assign::map_list_of;

typedef Registor_<base_Layer> LayerRegistor;
typedef Registor_<base_Layer>::Registry LayerRegistry;

// macros for adding individual instances to registry
#define ADD_TO_REGISTRY(Registor, NewInstance) (#NewInstance, &Registor::DerivedInstance<NewInstance>)
#define ADD_TO_LAYER_REGISTRY(NewInstance) ADD_TO_REGISTRY(LayerRegistor, NewInstance)

LayerRegistry g_layerRegistry = map_list_of
        ADD_TO_LAYER_REGISTRY( LayerZ )
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

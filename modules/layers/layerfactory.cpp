#include "layers/layerfactory.h"

#include <boost/assign/list_of.hpp>

#include "core/registor.h"

/** Whenever a new layer is imeplemented:
 *  1. include its header below
 *  2. Add it to the initialization of g_layerRegistry map.
 */
#include "layers/attentionwindow.h"
#include "layers/icp.h"
#include "layers/layer_y.h"
#include "layers/layer_z.h"
#include "encoding/populationcode.h"
#include "layers/saliencyitti.h"
#include "layers/triangulation.h"
#include "layers/weightedsum.h"

using boost::assign::map_list_of;

typedef Registor_<base_Layer> LayerRegistor;
typedef Registor_<base_Layer>::Registry LayerRegistry;

/** Macros for creating individual registry pair items
 *  credit: J. Turcot, T. Senechal, http://stackoverflow.com/questions/138600/initializing-a-static-stdmapint-int-in-c
 */
#define REGISTRY_PAIR(Registor, NewInstance) (#NewInstance, &Registor::DerivedInstance<NewInstance>)
#define LAYER_REGISTRY_PAIR(NewInstance) REGISTRY_PAIR(LayerRegistor, NewInstance)

LayerRegistry g_layerRegistry = map_list_of
        LAYER_REGISTRY_PAIR( AttentionWindow )
        LAYER_REGISTRY_PAIR( ICP )
        LAYER_REGISTRY_PAIR( LayerY )
        LAYER_REGISTRY_PAIR( LayerZ )
        LAYER_REGISTRY_PAIR( MutexPopulationCode )
        LAYER_REGISTRY_PAIR( SaliencyItti )
        LAYER_REGISTRY_PAIR( Triangulation )
        LAYER_REGISTRY_PAIR( WeightedSum )
        ; ///< <-- add new layer to registry here

LayerFactory::LayerFactory()
{
}

LayerRegistor::RegisteredTypeSharedPtr LayerFactory::CreateShared(const LayerType &type)
{
    return LayerRegistor::CreatePtrShared(g_layerRegistry, type);
}

LayerRegistor::RegisteredTypeSharedPtr LayerFactory::CreateShared(const LayerType &type,
                                                                          const LayerConfig &config,
                                                                          const LayerIONames &io)
{
    LayerRegistor::RegisteredTypeSharedPtr ptr = LayerFactory::CreateShared(type);
    ptr->Reset(config);
    ptr->IONames(io);
    return ptr;
}

#include "layers/layers.h"

#include <boost/assign/list_of.hpp>

#include "layers/layer_z.h"
#include "layers/saliencyitti.h"

using boost::assign::map_list_of;

// macros for adding individual instances to registry
#define ADD_TO_REGISTRY(Registor, NewInstance) (#NewInstance, &Registor::DerivedInstance<NewInstance>)
#define ADD_TO_LAYER_REGISTRY(NewInstance) ADD_TO_REGISTRY(LayerRegistor, NewInstance)

LayerRegistry g_layerRegistry = map_list_of
        ADD_TO_LAYER_REGISTRY( LayerZ )
        ADD_TO_LAYER_REGISTRY( SaliencyItti )
        ;


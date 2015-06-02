/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/layers/layerfactory.h"

#include <boost/assign/list_of.hpp>

#include "elm/core/exception.h"
#include "elm/core/registor.h"

/** Whenever a new layer is imeplemented:
 *  1. include its header below
 *  2. Add it to the initialization of g_layerRegistry map.
 */
#include "elm/layers/attentionwindow.h"
#include "elm/layers/gradassignment.h"
#include "elm/layers/graphcompatibility.h"
#include "elm/layers/icp.h"
#include "elm/layers/imagegradient.h"
#include "elm/layers/layer_y.h"
#include "elm/layers/medianblur.h"
#include "elm/layers/mlp.h"
#include "elm/encoding/populationcode_derivs/mutex_populationcode.h"
#include "elm/layers/saliencyitti.h"
#include "elm/layers/sinkhornbalancing.h"
#include "elm/layers/triangulation.h"
#include "elm/layers/weightedsum.h"

using boost::assign::map_list_of;
using namespace elm;

typedef Registor_<base_Layer> LayerRegistor;
typedef Registor_<base_Layer>::Registry LayerRegistry;

/** Macros for creating individual registry pair items
 *  credit: J. Turcot, T. Senechal, http://stackoverflow.com/questions/138600/initializing-a-static-stdmapint-int-in-c
 */
#define REGISTRY_PAIR(Registor, NewInstance) (#NewInstance, &Registor::DerivedInstance<NewInstance>)
#define LAYER_REGISTRY_PAIR(NewInstance) REGISTRY_PAIR(LayerRegistor, NewInstance)

LayerRegistry g_layerRegistry = map_list_of
        LAYER_REGISTRY_PAIR( AttentionWindow )
        LAYER_REGISTRY_PAIR( GradAssignment )
        LAYER_REGISTRY_PAIR( GraphCompatibility )
        LAYER_REGISTRY_PAIR( ICP )
        LAYER_REGISTRY_PAIR( ImageGradient )
        LAYER_REGISTRY_PAIR( LayerY )
        LAYER_REGISTRY_PAIR( MedianBlur )
        LAYER_REGISTRY_PAIR( MLP )
        LAYER_REGISTRY_PAIR( MutexPopulationCode )
        LAYER_REGISTRY_PAIR( SaliencyItti )
        LAYER_REGISTRY_PAIR( SinkhornBalancing )
        LAYER_REGISTRY_PAIR( Triangulation )
        LAYER_REGISTRY_PAIR( WeightedSum )
        ; ///< <-- add new layer to registry here

LayerFactory::LayerFactory()
{
    // run-time equivalent of LAYER_REGISTRY_PAIR( WeightedSum )
    //g_layerRegistry["WeightedSum"] = &LayerRegistor::DerivedInstance<WeightedSum>;
}

/** @todo eliminate need for checking not MSVC
 */
LayerRegistor::RegisteredTypeSharedPtr LayerFactory::CreateShared(const LayerType &type)
{
#if !_MSC_VER
    static_assert(std::is_same<LayerRegistor::RegisteredTypeSharedPtr, LayerShared >(), "Mismatching shared_ptr types.");
#endif // !_MSC_VER
    return LayerRegistor::CreatePtrShared(g_layerRegistry, type);
}

LayerRegistor::RegisteredTypeSharedPtr LayerFactory::CreateShared(const LayerType &type,
                                                                  const LayerConfig &config,
                                                                  const LayerIONames &io)
{
    LayerRegistor::RegisteredTypeSharedPtr ptr = LayerFactory::CreateShared(type);
    Init(ptr, config, io);
    return ptr;
}

bool LayerFactory::Exists(const LayerType &type) {

    return LayerRegistor::Find(g_layerRegistry, type);
}

void LayerFactory::Init(LayerShared &layer,
                        const LayerConfig &config,
                        const LayerIONames &io)
{
    if(!bool(layer)) {

        ELM_THROW_VALUE_ERROR("Undefined reference. Uninitialized pointer.");
    }
    layer->Reset(config);
    layer->IONames(io);
}

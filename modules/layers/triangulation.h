#ifndef SEM_LAYERS_TRIANGULATION_H_
#define SEM_LAYERS_TRIANGULATION_H_

#ifndef __WITH_PCL
    #warning "Disabling Triangulation layer due to no PCL support and defining it as a non-supported"
    #include "layers/layernotsupported.h"
    class Triangulation : public SEM_LAYER_NOT_SUPPORTED(Triangulation, "Building with PCL is required for supporting Triangulation layer.");
#else   // __WITH_PCL

#include <string>
#include "core/base_Layer.h"

/**
 * @brief The triangulation layer wraps around the greedy projection algorithm usign pcl @cite Marton09ICRA
 */
class Triangulation : public base_Layer
{
public:
    Triangulation();
};

#endif // __WITH_PCL

#endif // SEM_LAYERS_TRIANGULATION_H_

#ifndef _ELM_LAYERS_LAYERVERTEX_H_
#define _ELM_LAYERS_LAYERVERTEX_H_

#include <memory>

#include "elm/core/base_Layer.h"
#include "elm/core/layerconfig.h"
#include "elm/core/typedefs_fwd.h"

namespace elm {

/** @brief wrap layer information to store as vertex
 */
struct LayerVertex {

    LayerVertex()
        : is_active(false)
    {}

    /**
     * @brief Set necessary info
     * @param _cfg configuration
     * @param _io I/O names
     * @param _ptr pointer to instance
     */
    void Set(const LayerConfig &_cfg,
             const LayerIONames &_io,
             const LayerShared &_ptr);

    void Configure();

    LayerConfig cfg;    ///< configuration
    LayerIONames io;    ///< I/O
    LayerShared ptr;    ///< pointer to shared instance
    bool is_active;     ///< flag if active or not
};

} // namespace elm

#endif // _ELM_LAYERS_LAYERVERTEX_H_

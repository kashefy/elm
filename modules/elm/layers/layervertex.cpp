#include "elm/layers/layervertex.h"

using namespace elm;

LayerVertex::LayerVertex()
    : is_active(false)
{
}

void LayerVertex::Set(const LayerConfig &_cfg,
                      const LayerIONames &_io,
                      const LayerShared &_ptr) {

    cfg = _cfg;
    io  = _io;
    ptr = _ptr;
}

void LayerVertex::Configure() {

    ptr->Reset(cfg);
    ptr->IONames(io);
}

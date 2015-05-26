#include "elm/layers/layervertex.h"

#include "elm/layers/layerfactory.h"

using namespace elm;

LayerVertex::LayerVertex()
    : is_active(false)
{
}

void LayerVertex::Set(const std::string &_name,
                      const LayerConfig &_cfg,
                      const LayerIONames &_io,
                      const LayerShared &_ptr) {

    name = _name;
    cfg = _cfg;
    io  = _io;
    ptr = _ptr;
}

void LayerVertex::Configure() {

    if(!bool(ptr)) {

        ptr = LayerFactory::CreateShared(name, cfg, io);
    }
    else {

        LayerFactory::Init(ptr, cfg, io);
    }
}

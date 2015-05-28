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

void LayerVertex::to_ptree(PTree &p) const {

    PTree params = cfg.Params();
    p.put_child("params", params);

    PTree in;
    MapSS m;

    m = io.InputMap();
    for(MapSS::const_iterator itr=m.begin(); itr != m.end(); ++itr) {

        in.put(itr->first, itr->second);
    }

    PTree out, ptree_io;
    ptree_io.put_child("inputs", in);

    m = io.OutputMap();
    for(MapSS::const_iterator itr=m.begin(); itr != m.end(); ++itr) {

        out.put(itr->first, itr->second);
    }

    ptree_io.put_child("outputs", out);
    p.put_child("io", ptree_io);

    p.put("name", name);
}

#include "elm/layers/layervertex.h"

#include "elm/core/boost/ptree_utils.h"
#include "elm/core/inputname.h"
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
    MapSSToPTree(io.InputMap(), in);

    PTree out, ptree_io;
    ptree_io.put_child("inputs", in);

    MapSSToPTree(io.OutputMap(), out);

    ptree_io.put_child("outputs", out);
    p.put_child("io", ptree_io);

    p.put("name", name);
}

void LayerVertex::from_ptree(const PTree &p) {

    cfg.Params(p.get_child("params"));

    PTree ptree_io = p.get_child("io");

    {
        MapSS in;
        PTreeToMapSS(ptree_io.get_child("inputs"), in);

        for(MapSS::const_iterator itr=in.begin(); itr != in.end(); ++itr) {

            io.Input(itr->first, itr->second);
        }
    }
    {
        MapSS out;
        PTreeToMapSS(ptree_io.get_child("outputs"), out);

        for(MapSS::const_iterator itr=out.begin(); itr != out.end(); ++itr) {

            io.Output(itr->first, itr->second);
        }
    }

    name = p.get("name", name);
}

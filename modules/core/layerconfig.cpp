#include "core/layerconfig.h"

#include "core/exception.h"

using namespace std;

LayerConfig::LayerConfig()
{
}

void LayerConfig::Input(const string &key, const string &name)
{
   inputs_[key] = name;
}

void LayerConfig::Output(const string &key, const string &name)
{
    outputs_[key] = name;
}

void LayerConfig::Params(const PTree &params)
{
    params_ = params;
}

string LayerConfig::Input(const string &key) const
{
    string name;
    if(!Find(inputs_, key, name)) {

        SEM_THROW_KEY_ERROR("No name found for input key \'" + key + "\'");
    }
    return name;
}

string LayerConfig::Output(const string &key) const
{
    string name;
    if(!Find(outputs_, key, name)) {

        SEM_THROW_KEY_ERROR("No name found for input key \'" + key + "\'");
    }
    return name;
}

PTree LayerConfig::Params() const
{
    return params_;
}

bool LayerConfig::Find(const MapSS &map, const string &key, string &name) const
{
    MapSS::const_iterator itr = map.find(key);
    bool found = itr != map.end();
    if(found) {

        name = itr->second;
    }
    return found;
}

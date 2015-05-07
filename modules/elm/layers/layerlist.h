/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_LAYERS_LAYERLIST_H_
#define _ELM_LAYERS_LAYERLIST_H_

#include <string>

namespace std {

template <typename T> class shared_ptr; ///< convinience typedef for point template

} // namespace std

namespace elm {

class base_Layer;
class LayerConfig;
class LayerIONames;
class Signal;
class LayerListImpl;

class LayerList
{
public:
    ~LayerList();

    LayerList();

    void Add(std::shared_ptr<base_Layer> &layer, const LayerConfig &cfg, const LayerIONames &io);

    void AddOutput(const std::string &output_name);

    void RemoveOutput(const std::string &output_name);

    bool HasInputs(const Signal &s) const;

protected:
    LayerListImpl *impl_;
};

} // namespace elm

#endif // _ELM_LAYERS_LAYERLIST_H_

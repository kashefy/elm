/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/ts/layer_assertions.h"

using namespace std;
using namespace elm;

void elm::InitializeIONames(const MapIONames &io_pairs, const VecS& keys, LayerIONames& dst) {

    for(size_t i=0; i<keys.size(); i++) {

        IOName _v = io_pairs.at(keys[i]);

        if(_v.first == LayerIOKeyType::INPUT) {

            dst.Input(keys[i], _v.second);
        }
        else if(_v.first == LayerIOKeyType::OUTPUT) {

            dst.Output(keys[i], _v.second);
        }
    }
}

/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_CORE_LAYERIONAMES_H_
#define _ELM_CORE_LAYERIONAMES_H_

#include "elm/core/layerinputnames.h"
#include "elm/core/layeroutputnames.h"

template class std::map<std::string, std::string>;

namespace elm {

/**
 * @brief class for merging input/stimuli and output/response layer names
 */
class LayerIONames :
        public LayerInputNames,
        public LayerOutputNames
{
public:
};

} // namespace elm

#endif // _ELM_CORE_LAYERIONAMES_H_

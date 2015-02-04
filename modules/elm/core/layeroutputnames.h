/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_CORE_LAYEROUTPUTNAMES_H_
#define _ELM_CORE_LAYEROUTPUTNAMES_H_

#include "elm/core/typedefs_sfwd.h"

namespace elm {

/**
 * @brief class for encapsulating layer IO key-name pairs
 */
class LayerOutputNames
{
public:
    /**
     * @brief Set output key-name-pair. Overwrite if key exists.
     * @param key of feature required by layer
     * @param name und which the feature exists
     */
    void Output(const std::string &key, const std::string &name);

    /**
     * @brief Get name to output feature key
     * @param output feature key
     * @return name
     * @throw Key Error if key does not exist.
     */
    std::string Output(const std::string &key) const;

    /**
     * @brief Get name to optional output feature key
     * @param output feature key
     * @return name if exists, check existence using bool() and retrieve content via .get()
     */
    OptS OutputOpt(const std::string &key) const;

private:
    MapSS outputs_;
};

} // namespace elm

#endif // _ELM_CORE_LAYEROUTPUTNAMES_H_

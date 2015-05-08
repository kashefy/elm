/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_CORE_LAYERINPUTNAMES_H_
#define _ELM_CORE_LAYERINPUTNAMES_H_

#include "elm/core/typedefs_sfwd.h"

namespace elm {

class InputName;

/**
 * @brief class for encapsulating layer input/stimulus key-name pairs
 */
class LayerInputNames
{
public:
    LayerInputNames();

    /**
     * @brief Set input key-name-pair. Overwrite if key exists.
     * @param key of feature required by layer
     * @param name und which the feature exists
     */
    void Input(const std::string &key, const InputName &name);

    /**
     * @brief Get name to input feature key
     * @param input feature key
     * @return name
     * @throw Key Error if key does not exist.
     */
    InputName Input(const std::string &key) const;

    /**
     * @brief Get name to optional input feature key
     * @param input feature key
     * @return name if exists, check existence using bool() and retrieve content via .get()
     */
    OptS InputOpt(const std::string &key) const;

    const MapSS& InputMap() const;

protected:
    MapSS inputs_;
};

} // namespace elm

#endif // _ELM_CORE_LAYERINPUTNAMES_H_

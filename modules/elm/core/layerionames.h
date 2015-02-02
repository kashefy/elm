/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, Elm Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_CORE_LAYERIONAMES_H_
#define _ELM_CORE_LAYERIONAMES_H_

#include <map>

#include <boost/optional/optional_fwd.hpp>

#include "elm/core/inputname.h"

namespace elm {

typedef boost::optional<std::string> OptS;
typedef std::map<std::string, std::string> MapSS;

/**
 * @brief class for encapsulating layer IO key-name pairs
 */
class LayerIONames
{
public:
    /**
     * @brief Set input key-name-pair. Overwrite if key exists.
     * @param key of feature required by layer
     * @param name und which the feature exists
     */
    void Input(const std::string &key, const std::string &name);

    /**
     * @brief Set output key-name-pair. Overwrite if key exists.
     * @param key of feature required by layer
     * @param name und which the feature exists
     */
    void Output(const std::string &key, const std::string &name);

    /**
     * @brief Get name to input feature key
     * @param input feature key
     * @return name
     * @throw Key Error if key does not exist.
     */
    std::string Input(const std::string &key) const;

    /**
     * @brief Get name to optional input feature key
     * @param input feature key
     * @return name if exists, check existence using bool() and retrieve content via .get()
     */
    OptS InputOpt(const std::string &key) const;

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
    MapSS inputs_;
    MapSS outputs_;
};

} // namespace elm

#endif // _ELM_CORE_LAYERIONAMES_H_

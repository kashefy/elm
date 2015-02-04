/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_CORE_LAYERCONFIG_H_
#define _ELM_CORE_LAYERCONFIG_H_

#include <boost/property_tree/ptree.hpp>

#include "elm/core/layerionames.h"

namespace elm {

typedef boost::property_tree::ptree PTree;
typedef boost::optional<std::string> OptS;
typedef std::vector<std::string> VecS;
typedef std::map<std::string, std::string> MapSS;

class LayerConfig : public LayerIONames
{
public:
    /**
     * @brief Default Constructor
     */
    LayerConfig();

    /**
     * @brief Set layer parameters
     * @param params
     */
    void Params(const PTree &params);

    /**
     * @brief Get parameters
     * @return layer parameters
     */
    PTree Params() const;

private:
    PTree params_;
};

} // namespace elm

#endif // _ELM_CORE_LAYERCONFIG_H_

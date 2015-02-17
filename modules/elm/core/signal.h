/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_CORE_SIGNAL_H_
#define _ELM_CORE_SIGNAL_H_

#include "elm/core/typedefs_sfwd.h" // VecMat

#include "elm/core/featuredata.h"
#include "elm/core/signal_.h"

namespace elm {

typedef Signal_<FeatureData >::VecFeat_ VecFeatData; ///< convinience typedef for vector of FeatureData objects

/**
 * @brief The Signal class, a class for holding single and multiple samples of features
 */
class Signal : public Signal_<FeatureData >
{
public:
    ~Signal();

    Signal();

    /** @brief Get features as Mat under a given name
     * @param name
     * @return vector of features found under key represented as Mat objects
     * @todo Deprecate?
     */
    elm::VecMat operator [](const std::string &name) const;

    /** @brief Get most recent feature under a given name, short-circuit to returning Mat
      * @param name
      * @return most recent feature found under key
      */
    cv::Mat1f MostRecentMat1f(const std::string& name) const;

protected:
    typedef Signal_<FeatureData >::MapSVecFeat_ MapSVecFD; ///< convinience typedef for a map with string keys and VecFeatData values

};

} // namespace elm

#endif // _ELM_CORE_SIGNAL_H_

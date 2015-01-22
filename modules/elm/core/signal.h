#ifndef ELM_CORE_SIGNAL_H_
#define ELM_CORE_SIGNAL_H_

#include "sem/core/typedefs_sfwd.h" // VecMat

#include "sem/core/featuredata.h"
#include "sem/core/signal_.h"

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
    cv::Mat1f MostRecentMat(const std::string& name) const;

protected:
    typedef Signal_<FeatureData >::MapSVecFeat_ MapSVecFD; ///< convinience typedef for a map with string keys and VecFeatData values

};

#endif // ELM_CORE_SIGNAL_H_

#ifndef SEM_CORE_SIGNAL_H_
#define SEM_CORE_SIGNAL_H_

#include "core/typedefs.h"

#include "core/featuredata.h"
#include "core/signal_.h"

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
    VecMat operator [](const std::string &name) const;

    /** @brief Get most recent feature under a given name, short-circuit to returning Mat
      * @param name
      * @return most recent feature found under key
      */
    cv::Mat MostRecentMat(const std::string& name) const;

protected:
    typedef Signal_<FeatureData >::MapSVecFeat_ MapSVecFD; ///< convinience typedef for a map with string keys and VecFeatData values

};

#endif // SEM_CORE_SIGNAL_H_

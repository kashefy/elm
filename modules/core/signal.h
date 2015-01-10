#ifndef SEM_CORE_SIGNAL_H_
#define SEM_CORE_SIGNAL_H_

#include <map>
#include "core/typedefs.h"

#include <opencv2/core.hpp>

#include "core/featuredata.h"

typedef std::vector<FeatureData> VecFeatData; ///< convinience typedef for vector of FeatureData objects

/**
 * @brief The Signal class, a class for holding single and multiple samples of features
 */
class Signal
{
public:
    ~Signal();

    Signal();

    /**
     * @brief Clear everything
     */
    void Clear();

    /**
     * @brief Append signal feature to an existing key. Will add if it doesn't exist.
     * @param name or key
     * @param feature data to append
     */
    void Append(const std::string &name, const FeatureData &feature_data);

    /**
     * @brief Overloading Append method accepting MatExpr as input
     *
     * Mainly for resolving call ambiguities.
     *
     * @param name or key
     * @param feature data to append
     */
    void Append(const std::string &name, const cv::MatExpr &feature_data);

    /**
     * @brief Check if a signal exists under a given name
     * @param name/key
     * @return true if found
     */
    bool Exists(const std::string &name) const;

    /**
     * @brief Get a list of all available feature names
     * @return list of available feature names
     */
    VecS FeatureNames() const;

    /**
     * @brief Get features under a given name
     * @param name
     * @return vector of features found under key
     */
    VecFeatData GetFeatureData(const std::string &name) const;

    /** @brief Get features as Mat under a given name
     * @param name
     * @return vector of features found under key represented as Mat objects
     * @todo Deprecate?
     */
    VecMat operator [](const std::string &name) const;

    /** @brief Get most recent feature under a given name
      * @param name
      * @return most recent feature found under key
      */
    cv::Mat MostRecent(const std::string& name) const;

protected:
    typedef std::map<std::string, VecFeatData> MapSVecFD; ///< convinience typedef for a map with string keys and VecFeatData values

    MapSVecFD signals_; ///< encapuslated signal features
};

#endif // SEM_CORE_SIGNAL_H_

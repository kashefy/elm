#ifndef SEM_CORE_SIGNAL_H_
#define SEM_CORE_SIGNAL_H_

#include <map>
#include "core/typedefs.h"

#include <opencv2/core.hpp>

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
     * @brief Append signal to an existing key. Will add if it doesn't exist.
     * @param name or key
     * @param feature data to append
     */
    void Append(const std::string &name, const cv::Mat &feature_data);

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

    /** Get features under a given name
     * @param name
     * @return vector of features found under key
     */
    VecMat operator [](const std::string &name) const;

    /** Get most recent feature under a given name
      * @param name
      * @return most recent feature found under key
      */
    cv::Mat MostRecent(const std::string& name) const;

protected:
    std::map<std::string, VecMat> signals_; ///< encapuslated signals
};

#endif // SEM_CORE_SIGNAL_H_

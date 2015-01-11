#ifndef SEM_CORE_SIGNAL__H_
#define SEM_CORE_SIGNAL__H_

#include <map>

#include <opencv2/core.hpp>

#include "core/exception.h"
#include "core/stl.h"
#include "core/typedefs.h"

/**
 * @brief The template Signal class, a class for holding single and multiple samples of features
 */
template <class TFeat >
class Signal_
{
public:
    typedef std::vector<TFeat > VecFeat_;                   ///< convinience typedef for vector of FeatureData objects
    typedef std::map<std::string, VecFeat_> MapSVecFeat_;   ///< convinience typedef for a map with string keys and VecFeatData values

    ~Signal_() {}

    Signal_() {}

    /**
     * @brief Clear everything
     */
    void Clear()
    {
        signals_.clear();
    }

    /**
     * @brief Append signal feature to an existing key. Will add if it doesn't exist.
     * @param name or key
     * @param feature data to append
     */
    void Append(const std::string &name, const TFeat &feat)
    {
        typename MapSVecFeat_::iterator itr = signals_.find(name);
        if(itr != signals_.end()) { // found

            itr->second.push_back(feat);
        }
        else { // not found, new insertion

            signals_[name] = VecFeat_(1, feat);
        }
    }

    /**
     * @brief Check if a signal exists under a given name
     * @param name/key
     * @return true if found
     */
    bool Exists(const std::string &name) const
    {
        VecFeat_ tmp;
        return sem::Find(signals_, name, tmp);
    }

    /**
     * @brief Get a list of all available feature names
     * @return list of available feature names
     */
    VecS FeatureNames() const
    {
        VecS feature_names;
        for(typename MapSVecFeat_::const_iterator itr=signals_.begin();
            itr != signals_.end();
            ++itr) {

            feature_names.push_back(itr->first);
        }
        return feature_names;
    }

    /**
     * @brief Get features under a given name
     * @param name
     * @return vector of features found under key
     */
    VecFeat_ GetFeatureData(const std::string &name) const
    {
        typename MapSVecFeat_::const_iterator itr = signals_.find(name);
        if(itr != signals_.end()) { // found

            return itr->second;
        }
        else {
            std::stringstream s;
            s << "Feature \'" << name << "\' does not exist.";
            SEM_THROW_KEY_ERROR(s.str());
        }
    }

    /** @brief Get most recent feature under a given name
      * @param name
      * @return most recent feature found under key
      */
    TFeat MostRecent(const std::string& name) const
    {
        typename MapSVecFeat_::const_iterator itr = signals_.find(name);
        if(itr != signals_.end()) { // found

            size_t len = itr->second.size();
            if(len > 0) {

                return itr->second[len-1];
            }
            else {
                std::stringstream s;
                s << "Feature \'" << name << "\' is empty.";
                SEM_THROW_BAD_DIMS(s.str());
            }
        }
        else {
            std::stringstream s;
            s << "Feature \'" << name << "\' does not exist.";
            SEM_THROW_KEY_ERROR(s.str());
        }
    }

protected:
    MapSVecFeat_ signals_; ///< encapuslated signal features
};

#endif // SEM_CORE_SIGNAL__H_

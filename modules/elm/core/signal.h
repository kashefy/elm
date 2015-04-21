/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_CORE_SIGNAL_H_
#define _ELM_CORE_SIGNAL_H_

#include <memory>

#include "elm/core/typedefs_sfwd.h"
#include "elm/core/pcl/typedefs_fwd.h"

namespace elm {

class Signal_Impl;
class FeatureData;

/**
 * @brief The Signal class, a class for holding single and multiple samples of features
 */
class Signal
{
public:
    virtual ~Signal();

    Signal();

    void Clear();

    void Append(const std::string &name, const FeatureData &feat);

    void Append(const std::string &name, const cv::Mat1f &feat);

    void Append(const std::string &name, const VecMat1f &feat);

    void Append(const std::string &name, const SparseMat1f &feat);

    void Append(const std::string &name, const CloudXYZPtr &feat);

    bool Exists(const std::string &name) const;

    VecS FeatureNames() const;

    FeatureData MostRecent(const std::string& name) const;

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
    std::shared_ptr<Signal_Impl> impl_;
};

} // namespace elm

#endif // _ELM_CORE_SIGNAL_H_

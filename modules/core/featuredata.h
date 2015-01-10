#ifndef SEM_CORE_FEATUREDATA_H_
#define SEM_CORE_FEATUREDATA_H_

#include <memory>

#include <boost/variant.hpp>

#include <opencv2/core.hpp>

#include "core/featuredatavisitors.h"

/**
 * @brief The Feature data class.
 * Encapsulating a feature in multiple types and caching different represenations
 */
class FeatureData
{
public:
    /**
     * @brief Construct a new object with initial representation
     */
    template <class T>
    FeatureData(const T &t)
    {
        var_ = t;
    }

    /**
     * @brief Extract new representation of underlying feature data.
     * Utilizes cached conversion if applicable
     *
     * Implemented explcitiy for suported types.
     *
     * @return new feature data representation
     */
    template <class T>
    T get();

protected:
    /**
     * @brief Only keep overloaded constructors public
     */
    FeatureData();

    /**
     * @brief Initialization routine common to all constructors
     */
    void Init();

    /**
     * @brief Reset visitor cache
     */
    void Reset();

    boost::variant< cv::Mat1f, sem::CloudXYZ::Ptr > var_; ///< variant object to enable finite representations of a single feature data instance

    FeatDataVisitorMat_f visitor_mat_;      ///< visitor for converting to Mat objects
    FeatDataVisitorCloud visitor_cloud_;    ///< visitor for converting to pcl point clouds
};

#endif // SEM_CORE_FEATUREDATA_H_

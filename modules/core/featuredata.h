#ifndef SEM_CORE_FEATUREDATA_H_
#define SEM_CORE_FEATUREDATA_H_

#include <memory>

#include <boost/variant.hpp>

#include <opencv2/core.hpp>

#include "core/mat_utils.h"
#include "core/pcl_utils.h"

/**
 * @brief base class for caching feature data conversion
 */
class base_FeatDataConversionCache
{
public:
    virtual ~base_FeatDataConversionCache();

    /**
     * @brief Reset cache indicating a conversion took place
     */
    virtual void Reset();
protected:
    /**
     * @brief keep constructor protected, treat this as an interface
     */
    base_FeatDataConversionCache();
};

/**
 * @brief template class for different static visitors
 */
template <typename T>
class FeatDataVisitor_ :
        public boost::static_visitor<T >,
        public base_FeatDataConversionCache
{
};

/**
 * @brief visitor class for converting to Mat of floats
 */
class FeatDataVisitorMat_f :
        public FeatDataVisitor_<Mat_f >
{
public:
    Mat_f operator()(const Mat_f &m) const;

    Mat_f operator()(sem::CloudXYZ::Ptr &c) const;
};

/**
 * @brief visitor class for converting to pcl point cloud
 * And keeping track of when a heavy conversion (involving deep copy) occured
 */
class FeatDataVisitorCloud :
        public FeatDataVisitor_<sem::CloudXYZ::Ptr >
{
public:
    void Reset();

    sem::CloudXYZ::Ptr operator()(sem::CloudXYZ::Ptr &c);

    sem::CloudXYZ::Ptr operator()(const Mat_f &m);

protected:
    sem::CloudXYZ::Ptr c_; ///< internal reference for caching most recent conversion result
};

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
    FeatureData();

protected:
    /**
     * @brief Initialization routine common to all constructors
     */
    void Init();

    /**
     * @brief Reset visitor cache
     */
    void Reset();

    boost::variant< cv::Mat1f, sem::CloudXYZ::Ptr > var_; ///< variant object to enable finite representations of a single feature data instance

    FeatDataVisitorMat_f visitor_mat_;       ///< visitor for converting to Mat objects
    FeatDataVisitorCloud visitor_cloud_;    ///< visitor for converting to pcl point clouds
};

#endif // SEM_CORE_FEATUREDATA_H_

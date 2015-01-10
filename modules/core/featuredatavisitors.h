/** Define variant visitor classes for FeatureData conversions
  */
#ifndef SEM_CORE_FEATUREDATAVISITORS_H_
#define SEM_CORE_FEATUREDATAVISITORS_H_

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

#endif // SEM_CORE_FEATUREDATAVISITORS_H_

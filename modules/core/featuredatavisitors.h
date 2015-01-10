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
 * @brief template class for scalar POD static visitors
 */
template <typename T>
class FeatDataVisitorPOD_ :
        public boost::static_visitor<T >,
        public base_FeatDataConversionCache
{
public:
    T operator()(const Mat_f &m) const
    {
        size_t n = m.total();
        if(n != 1) {

            std::stringstream s;
            s << "Cannot convert " << n << "-element Mat to Scalar.";
            SEM_THROW_BAD_DIMS(s.str());
        }
        else {
            return static_cast<T>(m(0));
        }
    }

    template <class TPar>
    T operator()(TPar t) const
    {
        return static_cast<T>(t);
    }

#ifdef __WITH_PCL // PCL support required
    T operator()(sem::CloudXYZ::Ptr &c) const
    {
        SEM_THROW_TYPE_ERROR("Cannot convert point cloud to scalar.");
    }
#endif // __WITH_PCL
};

/**
 * @brief A do-nothing visitor class to use in place of unsupported visitors
 */
class FeatDataVisitorVoid :
        public FeatDataVisitor_<void>
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

    Mat_f operator()(float f) const;

    Mat_f operator()(int n) const;

    Mat_f operator()(uchar c) const;

#ifdef __WITH_PCL // this conversion requires PCL support
    Mat_f operator()(sem::CloudXYZ::Ptr &c) const;
#endif // __WITH_PCL
};

#ifdef __WITH_PCL // this visitor derived class definition requires PCL support

/**
 * @brief visitor class for converting to pcl point cloud
 * And keeping track of when a heavy conversion (involving deep copy) occured
 *
 * Requires PCL support.
 */
class FeatDataVisitorCloud :
        public FeatDataVisitor_<sem::CloudXYZ::Ptr >
{
public:
    void Reset();

    sem::CloudXYZ::Ptr operator()(sem::CloudXYZ::Ptr &c);

    sem::CloudXYZ::Ptr operator()(float f);

    sem::CloudXYZ::Ptr operator()(int n);

    sem::CloudXYZ::Ptr operator()(uchar c);

    sem::CloudXYZ::Ptr operator()(const Mat_f &m);

protected:
    sem::CloudXYZ::Ptr c_; ///< internal reference for caching most recent conversion result
};

#else // __WITH_PCL
    #warning "Unable to define FeatDataVisitorCloud visitor without PCL support."
#endif // __WITH_PCL

#endif // SEM_CORE_FEATUREDATAVISITORS_H_

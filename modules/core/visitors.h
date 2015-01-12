/** Define variant visitor classes for data type conversions
 * @todo switch Mat_f to basic Mat
  */
#ifndef SEM_CORE_VISITORS_H_
#define SEM_CORE_VISITORS_H_

#include <boost/variant.hpp>

#include <opencv2/core.hpp>

#include "core/mat_utils.h"
#include "core/pcl_utils.h"

/**
 * @brief base class for caching heavy data type conversions
 */
class base_ConversionCache
{
public:
    virtual ~base_ConversionCache();

    /**
     * @brief Reset cache indicating a conversion took place
     */
    virtual void Reset();
protected:
    /**
     * @brief keep constructor protected, treat this as an interface
     */
    base_ConversionCache();
};

/**
 * @brief template class for different static visitors
 */
template <typename T>
class Visitor_ :
        public boost::static_visitor<T >,
        public base_ConversionCache
{
};

/**
 * @brief template class for scalar POD static visitors
 */
template <typename T>
class VisitorPOD_ :
        public boost::static_visitor<T >,
        public base_ConversionCache
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

    T operator()(sem::VecVertices &vv) const
    {
        if(vv.size() == 1) {

            if(vv[0].vertices.size() == 1) {

                SEM_THROW_BAD_DIMS("Can only convert Vertices of size 1 to scalar.");
            }
            else {
                return static_cast<T>(vv[0].vertices[0]);
            }
        }
        else {

            SEM_THROW_BAD_DIMS("Can only convert VecVertices of size 1 to scalar.");
        }
    }
#endif // __WITH_PCL
};

/**
 * @brief A do-nothing visitor class to use in place of unsupported visitors
 */
class VisitorVoid :
        public Visitor_<void>
{
};

/**
 * @brief visitor class for converting to Mat of floats
 */
class VisitorMat_f :
        public Visitor_<Mat_f >
{
public:
    Mat_f operator()(const cv::Mat &m) const;

    Mat_f operator()(const Mat_f &m) const;

    Mat_f operator()(float f) const;

    Mat_f operator()(int n) const;

    Mat_f operator()(uchar c) const;

#ifdef __WITH_PCL // this conversion requires PCL support

    Mat_f operator()(sem::CloudXYZ::Ptr &c) const;

    Mat_f operator()(const sem::VecVertices &vv) const;

#endif // __WITH_PCL
};

#ifdef __WITH_PCL // the following visitor derived class definitions requires PCL support

/**
 * @brief visitor class for converting to pcl point cloud
 * And keeping track of when a heavy conversion (involving deep copy) occured
 *
 * Requires PCL support.
 */
class VisitorCloud :
        public Visitor_<sem::CloudXYZ::Ptr >
{
public:
    void Reset();

    sem::CloudXYZ::Ptr operator()(sem::CloudXYZ::Ptr &c);

    sem::CloudXYZ::Ptr operator()(const sem::VecVertices &vv);

    sem::CloudXYZ::Ptr operator()(float f);

    sem::CloudXYZ::Ptr operator()(int n);

    sem::CloudXYZ::Ptr operator()(uchar c);

    sem::CloudXYZ::Ptr operator()(const Mat_f &m);

protected:
    sem::CloudXYZ::Ptr c_; ///< internal reference for caching most recent conversion result
};

/**
 * @brief visitor class for converting to STL vector of PCL vertices
 * And keeping track of when a heavy conversion (involving deep copy) occured
 *
 * Requires PCL support.
 */
class VisitorVecVertices :
        public Visitor_<sem::VecVertices >
{
public:
    void Reset();

    sem::VecVertices operator()(const sem::VecVertices &vv);

    sem::VecVertices operator()(sem::CloudXYZ::Ptr &c);

    sem::VecVertices operator()(float f);

    sem::VecVertices operator()(int n);

    sem::VecVertices operator()(uchar c);

    sem::VecVertices operator()(const Mat_f &m);

protected:
    template <typename TScalar>
    void FromScalar(TScalar s) {

        Reset();
        pcl::Vertices v;
        v.vertices.push_back(static_cast<uint32_t>(s));
        vv_.push_back(v);
    }

    sem::VecVertices vv_;   ///< internal copy for caching most recent conversion result
};

#else // __WITH_PCL
    #warning "Unable to define VisitorCloud visitor without PCL support."
    #warning "Unable to define VisitorVecVertices visitor without PCL support."
#endif // __WITH_PCL

#endif // SEM_CORE_VISITORS_H_

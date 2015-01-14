#ifndef SEM_CORE_FEATUREDATA_H_
#define SEM_CORE_FEATUREDATA_H_

#include <memory>

#include <boost/variant.hpp>

#include "core/visitors/visitors.h"

/**
 * @brief The Feature data class.
 * Encapsulating a feature in multiple types and caching different represenations
 *
 * @todo map getter typename to visitor object at runtime
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

    template <class T>
    operator T()
    {
        return get<T>();
    }

    friend std::ostream& operator<<(std::ostream& os, const FeatureData &f);

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

    // variant and custom object visitors
#ifdef __WITH_PCL
    boost::variant< cv::Mat1f, sem::CloudXYZPtr, sem::VecVertices, float, int, uchar > var_; ///< variant object to enable finite representations of a single feature data instance

    VisitorCloud visitor_cloud_;    ///< visitor for converting to pcl point clouds
    VisitorVecVertices visitor_vv_; ///< visitor for converting to STL vector of PCL Vertices
#else
    boost::variant< cv::Mat1f, float, int, uchar > var_; ///< variant object to enable finite representations of a single feature data instance

    FeatDataVisitorVoid visitor_cloud_;     ///< place holder visitor that does nothing
    FeatDataVisitorVoid visitor_vv_;     ///< place holder visitor that does nothing
#endif // __WITH_PCL

    VisitorMat_f visitor_mat_;      ///< visitor for converting to Mat objects
};

std::ostream& operator<<(std::ostream& os, FeatureData& obj);

#endif // SEM_CORE_FEATUREDATA_H_

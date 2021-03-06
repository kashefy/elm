/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_CORE_FEATUREDATA_H_
#define _ELM_CORE_FEATUREDATA_H_

#include <iosfwd>

#include <boost/variant.hpp>

#include <opencv2/core/core.hpp>

#include "elm/core/typedefs_sfwd.h"
#include "elm/core/visitors/visitors.h"

extern template class cv::Mat_<float>;
extern template class cv::SparseMat_<float>;

namespace elm {

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
     * @brief Construct a new object with initially represented as a Mat
     */
    FeatureData(const cv::Mat &m)
    {
        var_ = static_cast<cv::Mat1f >(m);
    }

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
    boost::variant
    <
    cv::Mat1f,
    elm::SparseMat1f,
    elm::VecMat1f,
    elm::CloudXYZPtr,
    elm::CloudNrmlPtr,
    elm::CloudPtNrmlPtr,
    elm::VecVertices,
    float, int, uchar
    > var_; ///< variant object to enable finite representations of a single feature data instance

    elm::VisitorCloud_<pcl::PointXYZ> visitor_cloud_xyz_;       ///< visitor for converting to pcl point clouds
    elm::VisitorCloud_<pcl::Normal> visitor_cloud_nrml_;        ///< visitor for converting to pcl point clouds
    elm::VisitorCloud_<pcl::PointNormal> visitor_cloud_ptnrml_;   ///< visitor for converting to pcl point clouds

    VisitorVecVertices visitor_vv_; ///< visitor for converting to STL vector of PCL Vertices
#else
    boost::variant< cv::Mat1f, elm::SparseMat1f, elm::VecMat1f, float, int, uchar > var_; ///< variant object to enable finite representations of a single feature data instance

    VisitorVoid visitor_cloud_xyz_;     ///< place holder visitor that does nothing
    VisitorVoid visitor_cloud_nrml_;    ///< place holder visitor that does nothing
    VisitorVoid visitor_cloud_ptnrml_;  ///< place holder visitor that does nothing

    VisitorVoid visitor_vv_;            ///< place holder visitor that does nothing
#endif // __WITH_PCL

//    VisitorMat1f visitor_mat_;              ///< visitor for converting to Mat1f objects
//    VisitorSparseMat1f visitor_sparse_mat_; ///< visitor for converting to SparseMat1f objects
//    VisitorVecMat1f visitor_vm_;            ///< visitor for converting to VecMat1f objects
};

std::ostream& operator<<(std::ostream& os, FeatureData& obj);

} // namespace elm


#endif // _ELM_CORE_FEATUREDATA_H_

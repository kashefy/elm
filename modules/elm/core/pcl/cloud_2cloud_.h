/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_CORE_PCL_CLOUD__2CLOUD__H_
#define _ELM_CORE_PCL_CLOUD__2CLOUD__H_

#ifdef __WITH_PCL // the template class definitions require PCL support

#include <iostream>

#include "elm/core/exception.h"
#include "elm/core/pcl/cloud_.h"
#include "elm/core/pcl/point_traits.h"
#include "elm/core/pcl/typedefs_fwd.h"

namespace elm {
/**
 * @brief class for Point Cloud conversion of different point types
 */
template <class TPointSrc, class TPointDst>
class Cloud_2Cloud_
{
protected:
    typedef boost::shared_ptr<pcl::PointCloud<TPointSrc > > CloudTPSrcPtr;
    typedef boost::shared_ptr<pcl::PointCloud<TPointDst > > CloudTPDstPtr;

public:

    /**
     * @brief Convert point cloud of one type into another
     *
     * template specializations exsit for pcl::PointNormal to pcl::PointXYZ conversions
     *
     * @param[in] src cloud
     * @param[out] dst point cloud
     * @throws elm::ExceptionNotImpl when src and dst point types have have different footprints
     */
    static void Convert(CloudTPSrcPtr &src, CloudTPDstPtr &dst)
    {
//        size_t field_count_arg = PCLPointTraits_<TPointArg>::FieldCount();
//        size_t field_count_dst = PCLPointTraits_<TPoint>::FieldCount();

        size_t nb_floats_src = PCLPointTraits_<TPointSrc>::NbFloats();
        size_t nb_floats_dst = PCLPointTraits_<TPointDst>::NbFloats();

        if(nb_floats_src != nb_floats_dst) { // truncate

            ELM_THROW_NOT_IMPLEMENTED;
        }
        else {

            dst = Mat2PointCloud_<TPointDst >(PointCloud2Mat_<TPointSrc >(src));
        }
    }

protected:

};

// template specializations
template<>
void Cloud_2Cloud_<pcl::PointNormal, pcl::PointXYZ>::Convert(
        boost::shared_ptr<pcl::PointCloud<pcl::PointNormal > > &src,
        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ > > &dst
        );

template<>
void Cloud_2Cloud_<pcl::PointNormal, pcl::Normal>::Convert(
        boost::shared_ptr<pcl::PointCloud<pcl::PointNormal > > &src,
        boost::shared_ptr<pcl::PointCloud<pcl::Normal > > &dst
        );

} // namespace elm

#else // __WITH_PCL
    #warning "Unable to define cloud 2 cloud converter without PCL support."
#endif // __WITH_PCL

#endif // _ELM_CORE_PCL_CLOUD__2CLOUD__H_

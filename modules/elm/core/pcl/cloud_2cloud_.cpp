/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/core/pcl/cloud_2cloud_.h"

#ifdef __WITH_PCL // conversions from one point cloud to another require PCL support

using namespace pcl;

typedef boost::shared_ptr<pcl::PointCloud<PointXYZ > > CloudXYZPtr;
typedef boost::shared_ptr<pcl::PointCloud<PointNormal > > CloudPointNrmlPtr;
typedef boost::shared_ptr<pcl::PointCloud<Normal > > CloudNrmlPtr;

namespace elm {

template <class TPointSrc, class TPointDst>
void CloudTruncate(CloudPointNrmlPtr &src, CloudXYZPtr &dst)
{
    dst.reset(new PointCloud<TPointDst>(src->width, src->height));

    size_t nb_floats_src = PCLPointTraits_<TPointSrc>::NbFloats();
    size_t nb_floats_dst = PCLPointTraits_<TPointDst>::NbFloats();

    size_t step_size = sizeof(float)*std::min(nb_floats_src, nb_floats_dst);

    typename pcl::PointCloud<TPointSrc>::iterator itr_src = src->begin();
    typename pcl::PointCloud<TPointDst>::iterator itr_dst = dst->begin();
    while(itr_src != src->end()) {
        \
        memcpy(&(*itr_src), &(*itr_dst), step_size); // dangerous?

        ++itr_src;
        ++itr_dst;
    }
}

/**
 * @brief template specialization for converting point cloud of pcl::PointNormal points to cloud of pcl::PointXYZ points
 *
 * Involves a deep copy of point data.
 *
 * @todo: make this the template implementation instead of throwing elm::ExceptionNotImpl?
 *
 */
template<>
void Cloud_2Cloud_<PointNormal, PointXYZ>::Convert(CloudPointNrmlPtr &src, CloudXYZPtr &dst)
{
    CloudTruncate<PointNormal, PointXYZ>(src, dst);
}

/**
 * @brief The Normal data is offsetted inside the PointNormal data
 */
template<>
void Cloud_2Cloud_<PointNormal, Normal>::Convert(CloudPointNrmlPtr &src, CloudNrmlPtr &dst)
{
    dst.reset(new PointCloud<Normal>(src->width, src->height));

     size_t nb_floats_src = PCLPointTraits_<PointNormal>::NbFloats();
     size_t nb_floats_dst = PCLPointTraits_<Normal>::NbFloats();

     size_t step_size = sizeof(float)*nb_floats_dst;
     size_t src_float_offset = nb_floats_src-nb_floats_dst;

     typename pcl::PointCloud<PointNormal>::iterator itr_src = src->begin();
     typename pcl::PointCloud<Normal>::iterator itr_dst = dst->begin();
     while(itr_src != src->end()) {

         float* pt_src_data_ptr = reinterpret_cast<float*>(&(*itr_src))+src_float_offset;
         memcpy(pt_src_data_ptr, &(*itr_dst), step_size); // dangerous?

         ++itr_src;
         ++itr_dst;
     }
}


}

#endif // __WITH_PCL

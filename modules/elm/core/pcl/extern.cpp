/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifdef __WITH_PCL

#include "elm/core/pcl/cloud_.h"
#include "elm/core/pcl/vertices.h"

template class pcl::PointCloud<pcl::PointXYZ >;
template class pcl::PointCloud<pcl::Normal >;
template class pcl::PointCloud<pcl::PointNormal >;

template class boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ > >;
template class boost::shared_ptr<pcl::PointCloud<pcl::Normal > >;
template class boost::shared_ptr<pcl::PointCloud<pcl::PointNormal > >;

template class std::vector<pcl::Vertices >;

#endif // __WITH_PCL

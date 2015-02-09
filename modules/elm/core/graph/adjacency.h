/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_CORE_GRAPH_ADJACENCY_H_
#define _ELM_CORE_GRAPH_ADJACENCY_H_

#include "elm/core/typedefs_fwd.h"
#include "elm/core/pcl/typedefs_fwd.h"

namespace elm {

class Graph;

#ifdef __WITH_PCL

/**
 * @brief Get Adjacency matrix for triangulated point cloud
 * @param cld point cloud
 * @param t vertices of point cloud triangulation
 * @param dst dense adjacency matrix
 * @throws elm::ExceptionBadDims if verticies don't add up to form triangle
 * @throws elm::ExceptionKeyError if a triangle vertex is beyond cloud points
 * @todo figure out speed difference between dense and sparse overload
 */
void TriangulatedCloudToAdjacency(const CloudXYZPtr &cld, const Triangles &t, cv::Mat1f &dst);

/**
 * @brief Get Adjacency matrix for triangulated point cloud
 * @param cld point cloud
 * @param t vertices of point cloud triangulation
 * @param dst sparse adjacency matrix
 * @throws elm::ExceptionBadDims if verticies don't add up to form triangle
 * @throws elm::ExceptionKeyError if a triangle vertex is beyond cloud points
 * @todo figure out speed difference between dense and sparse overload
 */
void TriangulatedCloudToAdjacency(const CloudXYZPtr &cld, const Triangles &t, elm::SparseMat1f &dst);

#endif // __WITH_PCL


} // namespace elm

#endif // _ELM_CORE_GRAPH_ADJACENCY_H_

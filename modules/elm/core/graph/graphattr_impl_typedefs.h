/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_GRAPHATTR_IMPL_TYPEDEFS_H_
#define _ELM_GRAPHATTR_IMPL_TYPEDEFS_H_

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>

#include <opencv2/core/core.hpp>

#include "elm/core/typedefs_fwd.h"

extern template class cv::Mat_<float>;

namespace elm {

typedef float EdgeWeight;
typedef boost::property<boost::edge_weight_t, EdgeWeight> EdgeWeightProp;

typedef int VtxColor;
typedef cv::Mat1f VtxIdx2;
typedef boost::property<boost::vertex_color_t, VtxColor,
        boost::property<boost::vertex_index2_t, VtxIdx2 > >
        VtxProp;

typedef boost::adjacency_list<boost::setS, boost::listS, boost::undirectedS, VtxProp, EdgeWeightProp> GraphAttrType;

typedef boost::graph_traits<GraphAttrType> GraphAttrTraits;
typedef GraphAttrTraits::edge_iterator edge_iter;
typedef GraphAttrTraits::vertex_descriptor VtxDescriptor;

} // namespace elm

#endif // _ELM_GRAPHATTR_IMPL_TYPEDEFS_H_

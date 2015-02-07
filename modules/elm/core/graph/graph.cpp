/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/core/graph/graph.h"

#include "elm/core/graph/graph_impl.h"

using namespace elm;

Graph::~Graph()
{
    delete impl;
}

Graph::Graph()
    : impl(new Graph_Impl())
{
}

Graph::Graph(int nb_vertices)
    : impl(new Graph_Impl(nb_vertices))
{
}

#ifdef __WITH_PCL

Graph::Graph(const CloudXYZPtr &cld, const Triangles &t)
    : impl(new Graph_Impl(cld, t))
{
}

#endif // __WITH_PCL

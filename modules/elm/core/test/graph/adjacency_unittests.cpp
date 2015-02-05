/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/core/graph/adjacency.h"

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>

#ifdef __WITH_PCL

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/Vertices.h>

#include "elm/core/exception.h"
#include "elm/core/pcl/triangle_utils.h"
#include "elm/ts/mat_assertions.h"

using namespace cv;
using namespace pcl;
using namespace elm;

namespace {


class AdjacencyTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {

    }

    // members

};

TEST_F(AdjacencyTest, DummyTest)
{
    CloudXYZPtr cld;
    Triangles tri;
    cv::Mat1f adj;
    EXPECT_THROW(TriangulatedCloudToAdjacencyX(cld, tri, adj), ExceptionNotImpl);
}

typedef boost::property<boost::edge_weight_t, float> EdgeWeightProperty;
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, boost::no_property, EdgeWeightProperty> Graph;
typedef boost::graph_traits<Graph>::edge_iterator edge_iter;

void TriangulatedCloudToAdjacencyX(const CloudXYZPtr &cld, const Triangles &t, Graph &dst)
{
    //int nb_vertices = static_cast<int>(cld->size());

    for(Triangles::const_iterator itr=t.begin(); itr!=t.end(); ++itr) {

        const Vertices V = *itr;

        if(V.vertices.size() < size_t(3)) {

            ELM_THROW_BAD_DIMS("triangle with less than 3 vertices.");
        }

        //foreach vertex
        uint32_t v0 = V.vertices[0];
        uint32_t v1 = V.vertices[1];
        uint32_t v2 = V.vertices[2];

        Mat1f e_triangle = TriangleEdges(cld->points.at(v0),
                                         cld->points.at(v1),
                                         cld->points.at(v2));

        EdgeWeightProperty e = e_triangle(0);
        add_edge(v0, v1, e, dst);

        e = e_triangle(1);
        add_edge(v0, v2, e, dst);

        e = e_triangle(2);
        add_edge(v1, v2, e, dst);
    }

    // iterate over all edges
//    boost::property_map<Graph, boost::edge_weight_t>::type EdgeWeightMap = get(boost::edge_weight_t(), g);

//    for(std::pair<edge_iter, edge_iter> edge_pair = edges(g);
//        edge_pair.first != edge_pair.second;
//        ++edge_pair.first) {

//        std::cout << EdgeWeightMap[*edge_pair.first] << " ";
//    }

    //m_g->get_edge(u, v);
}

TEST_F(AdjacencyTest, AdjacencyBoostGraph)
{
    CloudXYZPtr cld;
    Triangles tri;

    int nb_vertices = static_cast<int>(cld->size());
    Graph g(nb_vertices);

    TriangulatedCloudToAdjacencyX(cld, tri, g);
}

} // annonymous namespace for tests

#endif // __WITH_PCL

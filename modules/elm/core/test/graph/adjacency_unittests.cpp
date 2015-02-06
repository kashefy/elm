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
        cld_.reset(new CloudXYZ);
        tri_.clear();
    }

    // members
    CloudXYZPtr cld_;
    Triangles tri_;
};

TEST_F(AdjacencyTest, Empty)
{
    // both cloud and vertices empty
    cld_->clear();
    tri_.clear();

    cv::Mat1f adj;
    EXPECT_NO_THROW(TriangulatedCloudToAdjacencyX(cld_, tri_, adj));
    EXPECT_TRUE(adj.empty());

    // cloud empty
    cld_->clear();

    Vertices v;
    v.vertices.push_back(uint32_t(1));
    v.vertices.push_back(uint32_t(2));
    v.vertices.push_back(uint32_t(3));
    tri_.push_back(v);

    EXPECT_THROW(TriangulatedCloudToAdjacencyX(cld_, tri_, adj), ExceptionKeyError);


    // triangles empty
    cld_->push_back(PointXYZ(1.f, 2.f, 3.f));

    tri_.clear();
    EXPECT_NO_THROW(TriangulatedCloudToAdjacencyX(cld_, tri_, adj));
    EXPECT_MAT_DIMS_EQ(adj, Size2i(1, 1));
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
    int nb_vertices = static_cast<int>(cld_->size());
    Graph g(nb_vertices);

    TriangulatedCloudToAdjacencyX(cld_, tri_, g);
}

} // annonymous namespace for tests

#endif // __WITH_PCL

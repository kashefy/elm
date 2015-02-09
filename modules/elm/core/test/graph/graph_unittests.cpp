/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/core/graph/graph.h"

#include "gtest/gtest.h"

#include <opencv2/core.hpp>

#ifdef __WITH_PCL

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/Vertices.h>

#endif // __WITH_PCL

#include "elm/core/debug_utils.h"
#include "elm/core/exception.h"
#include "elm/core/cv/mat_utils_inl.h"
#include "elm/ts/mat_assertions.h"

using namespace cv;
using namespace elm;

namespace {

class GraphConstructTest : public ::testing::Test
{
protected:
};

TEST_F(GraphConstructTest, Consturct)
{
    EXPECT_NO_THROW(Graph to);
    EXPECT_NO_THROW(Graph(0));
    EXPECT_NO_THROW(Graph(1));
}

TEST_F(GraphConstructTest, Num_Vertices)
{
    for(int i=0; i<10; i++) {

        Graph to(i);
        EXPECT_EQ(size_t(i), to.num_vertices());
    }
}

TEST_F(GraphConstructTest, Destruct)
{
    Graph *to = new Graph();
    EXPECT_NO_THROW(delete to);
}

#ifdef __WITH_PCL

using namespace pcl;

class GraphTriangulatedConstructTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        cld_.reset(new CloudXYZ);
        tri_.clear();
    }

    // members
    CloudXYZPtr cld_;   ///< point cloud
    Triangles tri_;     ///< triangulated vertices
};

TEST_F(GraphTriangulatedConstructTest, Invalid_too_few_vertices)
{
    cld_->clear();
    cld_->push_back(PointXYZ(0.f, 0.f, 0.f));
    cld_->push_back(PointXYZ(1.f, 1.f, 1.f));
    cld_->push_back(PointXYZ(2.f, 2.f, 2.f));

    for(uint32_t i=0; i<5; i++) {

        tri_.clear();
        Vertices v;
        for(uint32_t j=0; j<i; j++) {

            v.vertices.push_back(i%static_cast<uint32_t>(cld_->size()));
        }
        tri_.push_back(v);

        if(i < 3) {

            EXPECT_THROW(Graph g(cld_, tri_), ExceptionBadDims);
        }
        else {

            EXPECT_NO_THROW(Graph g(cld_, tri_));
        }
    }
}

TEST_F(GraphTriangulatedConstructTest, Invalid_vertices_out_of_bounds)
{
    cld_->clear();
    cld_->push_back(PointXYZ(0.f, 0.f, 0.f));
    cld_->push_back(PointXYZ(1.f, 1.f, 1.f));
    cld_->push_back(PointXYZ(2.f, 2.f, 2.f));

    tri_.clear();
    Vertices v;
    v.vertices.push_back(uint32_t(0));
    v.vertices.push_back(uint32_t(1));
    v.vertices.push_back(static_cast<uint32_t>(cld_->size()));
    tri_.push_back(v);

    EXPECT_THROW(Graph g(cld_, tri_), ExceptionKeyError);
}

/**
 * @brief Test with either empty cloud or empty triangles, or both empty
 */
TEST_F(GraphTriangulatedConstructTest, Empty)
{
    // both cloud and vertices empty
    cld_->clear();
    tri_.clear();

    {
        Graph g(cld_, tri_);
        EXPECT_EQ(size_t(0), g.num_vertices());
    }

    // cloud empty
    cld_->clear();

    Vertices v;
    v.vertices.push_back(uint32_t(1));
    v.vertices.push_back(uint32_t(2));
    v.vertices.push_back(uint32_t(3));
    tri_.push_back(v);

    EXPECT_THROW(Graph g(cld_, tri_), ExceptionKeyError);

    // triangles empty
    cld_->push_back(PointXYZ(1.f, 2.f, 3.f));

    tri_.clear();
    {
        Graph g(cld_, tri_);
        EXPECT_EQ(size_t(1), g.num_vertices());
    }
}

TEST_F(GraphTriangulatedConstructTest, Dims)
{
    for(int i=1; i<=11; i++) {

        CloudXYZPtr cld(new CloudXYZ);
        for(int j=0; j<i; j++) {

            cld->push_back(PointXYZ(1.f, 1.f, 1.f));
        }

        Triangles tri;

        {
            Graph g(cld, tri);
            EXPECT_EQ(size_t(i), g.num_vertices())
                    << "Unexpected no. of vertices for graph from triangulated point cloud.";
        }

        // fake triangles:
        for(int j=0; j<i; j++) {

            Vertices v;
            // vertices don't really mena anything, we may as well have self connections
            v.vertices.push_back(j);
            (j-1 >= 0) ? v.vertices.push_back(j-1) :
                         v.vertices.push_back(j);

            (j-2 >= 0) ? v.vertices.push_back(j-2) :
                         v.vertices.push_back(j);

            tri.push_back(v);
        }

        {
            Graph g(cld, tri);
            EXPECT_EQ(size_t(i), g.num_vertices())
                    << "Unexpected no. of vertices for graph from triangulated point cloud."
                    << " After adding triangles.";
        }
    }
}

class GraphTriangulatedTest : public GraphTriangulatedConstructTest
{
protected:
    virtual void SetUp()
    {
        GraphTriangulatedConstructTest::SetUp();

        const int NB_VERTICES = 5;
        const int NB_TRIANGLES = NB_VERTICES-1;

        // generate fake points
        for(int i=0; i<NB_VERTICES; i++) {

            float x = randu<float>();
            float y = randu<float>();
            float z = randu<float>();

            cld_->push_back(PointXYZ(x, y, z));
        }

        // fake triangles around fake cloud
        Mat1i v_range = ARange_<int>(0, NB_VERTICES, 1);
        for(int i=0; i<NB_TRIANGLES; i++) {

            randShuffle(v_range);
            Vertices v;
            for(int j=0; j<3; j++) {

                v.vertices.push_back(static_cast<uint32_t>(v_range(j)));
            }

            tri_.push_back(v);
        }
    }
};

TEST_F(GraphTriangulatedTest, AccessEdgeWeight)
{
    cld_->clear();
    tri_.clear();

    {
        cld_->push_back(PointXYZ(0.f, 0.f, 0.f));
        cld_->push_back(PointXYZ(1.f, 1.f, 1.f));
        cld_->push_back(PointXYZ(2.f, 2.f, 2.f));
        cld_->push_back(PointXYZ(3.f, 3.f, 3.f));
    }
    {
        Vertices v;
        v.vertices.push_back(u_int32_t(0));
        v.vertices.push_back(u_int32_t(1));
        v.vertices.push_back(u_int32_t(2));
        tri_.push_back(v);
    }

    Graph to(cld_, tri_);
    EXPECT_EQ(size_t(4), to.num_vertices());

    EXPECT_FLOAT_EQ(0.f, to(0, 3)) << "this vertex does not have any connections.";

    EXPECT_FLOAT_EQ(to(0, 1), static_cast<float>(sqrt(3.)));
    EXPECT_FLOAT_EQ(to(0, 2), static_cast<float>(2.*sqrt(3.)));
    EXPECT_FLOAT_EQ(to(1, 2), to(0, 1));
}

TEST_F(GraphTriangulatedTest, Adjacency)
{
    Graph to(cld_, tri_);
    EXPECT_EQ(cld_->size(), to.num_vertices())
            << "Unexpected no. of vertices for graph constructed from triangulated point cloud.";

    const int NB_VERTICES = static_cast<int>(to.num_vertices());
    // check adjacency matrix is symmetric
    for(int i=0; i<NB_VERTICES; i++) {

        for(int j=0; j<NB_VERTICES; j++) {

            EXPECT_FLOAT_EQ(to(i, j), to(j, i))
                    << "Non-symmetric edge weight between vertices (" << i << ", " << j << ")";
        }
    }

    // check edges are all positive
    for(int i=0; i<NB_VERTICES; i++) {

        for(int j=0; j<NB_VERTICES; j++) {

            EXPECT_GE(to(i, j), 0) << "Edge weights must be >=0.";
        }
    }

    // check edge values
    for(int i=0; i<NB_VERTICES; i++) {

        PointXYZ pi = cld_->at(i);

        for(int j=0; j<NB_VERTICES; j++) {

            if(i==j) {
                EXPECT_FLOAT_EQ(0.f, to(i, j)) << "Self connection found for vertex i=" << i;
            }
            else if(to(i, j) == 0.f) {

                // verify that this edge does not exist
                // does edge exist?

                for(Triangles::const_iterator itr=tri_.begin(); itr!=tri_.end(); ++itr) {

                    std::vector<uint32_t> verts = (*itr).vertices;
                    std::vector<uint32_t>::iterator itr_i = find(verts.begin(), verts.end(), static_cast<uint32_t>(i));
                    std::vector<uint32_t>::iterator itr_j = find(verts.begin(), verts.end(), static_cast<uint32_t>(j));

                    EXPECT_FALSE((itr_i != verts.end()) && (itr_j != verts.end()) && itr_i != itr_j)
                            << "Zero weight for existing edge between vertices "
                            << "i=" << i << ", j=" << j;
                }
            }
            else {

                // verify edge weight
                PointXYZ pj = cld_->at(j);

                // calculate distance between two points
                PointXYZ diff(pj.x-pi.x, pj.y-pi.y, pj.z-pi.z);
                PointXYZ diff_sq(diff.x*diff.x, diff.y*diff.y, diff.z*diff.z);

                EXPECT_FLOAT_EQ(sqrt(diff_sq.x+diff_sq.y+diff_sq.z), to(i, j))
                        << "Unexpected edge value.";
            }
        }
    }
}

TEST_F(GraphTriangulatedTest, AdjacencyMatDense)
{
    Graph to(cld_, tri_);
    EXPECT_EQ(cld_->size(), to.num_vertices())
            << "Unexpected no. of vertices for graph constructed from triangulated point cloud.";

    const int NB_VERTICES = static_cast<int>(to.num_vertices());

    Mat1f adj;
    to.AdjacencyMat(adj);

    EXPECT_MAT_DIMS_EQ(adj, Size2i(NB_VERTICES, NB_VERTICES));

    // check adjacency matrix is symmetric
    for(int i=0; i<NB_VERTICES; i++) {

        for(int j=0; j<NB_VERTICES; j++) {

            EXPECT_FLOAT_EQ(to(i, j), adj(i, j))
                    << "Non-symmetric edge weight between vertices (" << i << ", " << j << ")";
        }
    }
}

TEST_F(GraphTriangulatedTest, AdjacencyMatDense_ref)
{
    Graph to(cld_, tri_);
    EXPECT_EQ(cld_->size(), to.num_vertices())
            << "Unexpected no. of vertices for graph constructed from triangulated point cloud.";
    ASSERT_GT(to.num_vertices(), size_t(0));

    Mat1f adj;
    uchar* data_ptr = adj.data;
    to.AdjacencyMat(adj);

    EXPECT_NE(data_ptr, adj.data) << "pointer to data has not changed.";

    data_ptr = adj.data;
    to.AdjacencyMat(adj);
    EXPECT_EQ(data_ptr, adj.data) << "pointer to data changed.";
}

TEST_F(GraphTriangulatedTest, AdjacencyMatSparse)
{
    Graph to(cld_, tri_);
    EXPECT_EQ(cld_->size(), to.num_vertices())
            << "Unexpected no. of vertices for graph constructed from triangulated point cloud.";

    const int NB_VERTICES = static_cast<int>(to.num_vertices());

    SparseMat1f adj;
    to.AdjacencyMat(adj);
    EXPECT_EQ(NB_VERTICES, adj.size()[0]);
    EXPECT_EQ(NB_VERTICES, adj.size()[1]);

    // check adjacency matrix is symmetric
    for(int i=0; i<NB_VERTICES; i++) {

        for(int j=0; j<NB_VERTICES; j++) {

            EXPECT_FLOAT_EQ(to(i, j), adj.ref(i, j))
                    << "Non-symmetric edge weight between vertices (" << i << ", " << j << ")";
        }
    }
}

#endif // __WITH_PCL

} // annonymous namespace for unit tests


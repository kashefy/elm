/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/core/graph/adjacency.h"

#ifdef __WITH_PCL

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/Vertices.h>

#include "elm/core/debug_utils.h"

#include "elm/core/exception.h"
#include "elm/core/cv/mat_utils_inl.h"
#include "elm/core/graph/graph.h"
#include "elm/core/pcl/triangle_utils.h"
#include "elm/ts/mat_assertions.h"

using namespace cv;
using namespace pcl;
using namespace elm;

namespace {

/**
 * @brief class for testing routines that compute adjacency matrices from graphs
 * In particular, graphs around triangulated point clouds
 */
class AdjacencyDenseTest : public ::testing::Test
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

TEST_F(AdjacencyDenseTest, Invalid_too_few_vertices)
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

        cv::Mat1f adj;
        if(i < 3) {

            EXPECT_THROW(TriangulatedCloudToAdjacency(cld_, tri_, adj), ExceptionBadDims);
        }
        else {

            EXPECT_NO_THROW(TriangulatedCloudToAdjacency(cld_, tri_, adj));
        }
    }
}

TEST_F(AdjacencyDenseTest, Invalid_vertices_out_of_bounds)
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

    cv::Mat1f adj;
    EXPECT_THROW(TriangulatedCloudToAdjacency(cld_, tri_, adj), ExceptionKeyError);
}

TEST_F(AdjacencyDenseTest, Empty)
{
    // both cloud and vertices empty
    cld_->clear();
    tri_.clear();

    cv::Mat1f adj;
    EXPECT_NO_THROW(TriangulatedCloudToAdjacency(cld_, tri_, adj));
    EXPECT_TRUE(adj.empty());

    // cloud empty
    cld_->clear();

    Vertices v;
    v.vertices.push_back(uint32_t(1));
    v.vertices.push_back(uint32_t(2));
    v.vertices.push_back(uint32_t(3));
    tri_.push_back(v);

    EXPECT_THROW(TriangulatedCloudToAdjacency(cld_, tri_, adj), ExceptionKeyError);

    // triangles empty
    cld_->push_back(PointXYZ(1.f, 2.f, 3.f));

    tri_.clear();
    EXPECT_NO_THROW(TriangulatedCloudToAdjacency(cld_, tri_, adj));
    EXPECT_MAT_DIMS_EQ(adj, Size2i(1, 1));
}

TEST_F(AdjacencyDenseTest, Dims)
{
    for(int i=1; i<=11; i++) {

        CloudXYZPtr cld(new CloudXYZ);
        for(int j=0; j<i; j++) {

            cld->push_back(PointXYZ(1.f, 1.f, 1.f));
        }

        Triangles tri;

        {
            Mat1f adj;
            EXPECT_NO_THROW(TriangulatedCloudToAdjacency(cld, tri, adj));
            EXPECT_MAT_DIMS_EQ(adj, Size2i(i, i))
                    << "Unexpected dimensions for adjacency matrix from triangulated point cloud.";
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
            Mat1f adj;
            EXPECT_NO_THROW(TriangulatedCloudToAdjacency(cld, tri, adj));
            EXPECT_MAT_DIMS_EQ(adj, Size2i(i, i))
                    << "Unexpected dimensions for adjacency matrix from triangulated point cloud."
                    << " After adding triangles.";
        }
    }
}

TEST_F(AdjacencyDenseTest, AdjacencyMatValues)
{
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

    Mat1f adj;
    TriangulatedCloudToAdjacency(cld_, tri_, adj);
    EXPECT_MAT_DIMS_EQ(adj, Size2i(NB_VERTICES, NB_VERTICES))
            << "Unexpected dimensions for adjacency matrix from triangulated point cloud.";

    // check adjacency matrix is symmetric
    EXPECT_MAT_EQ(adj, adj.t()) << "Adjacency matrix is not symmetric.";

    // check edges are all positive
    for(int i=0; i<NB_VERTICES; i++) {

        for(int j=0; j<NB_VERTICES; j++) {

            EXPECT_GE(adj(i, j), 0) << "Edge weights must be >=0.";
        }
    }

    // check edge values
    for(int i=0; i<NB_VERTICES; i++) {

        PointXYZ pi = cld_->at(i);

        for(int j=0; j<NB_VERTICES; j++) {

            if(i==j) {
                EXPECT_FLOAT_EQ(0.f, adj(i, j)) << "Self connection found for vertex i=" << i;
            }
            else if(adj(i, j) == 0.f) {

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

                EXPECT_FLOAT_EQ(sqrt(diff_sq.x+diff_sq.y+diff_sq.z), adj(i, j))
                        << "Unexpected edge value.";
            }
        }
    }
}

class AdjacencySparseTest : public AdjacencyDenseTest
{
};

TEST_F(AdjacencySparseTest, Invalid_too_few_vertices)
{
    // both cloud and vertices empty
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

        SparseMat1f adj;
        if(i < 3) {

            EXPECT_THROW(TriangulatedCloudToAdjacency(cld_, tri_, adj), ExceptionBadDims);
        }
        else {

            EXPECT_NO_THROW(TriangulatedCloudToAdjacency(cld_, tri_, adj));
        }
    }
}

TEST_F(AdjacencySparseTest, Invalid_vertices_out_of_bounds)
{
    // both cloud and vertices empty
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

    SparseMat1f adj;
    EXPECT_THROW(TriangulatedCloudToAdjacency(cld_, tri_, adj), ExceptionKeyError);
}

TEST_F(AdjacencySparseTest, Empty)
{
    // both cloud and vertices empty
    cld_->clear();
    tri_.clear();

    SparseMat1f adj;
    EXPECT_NO_THROW(TriangulatedCloudToAdjacency(cld_, tri_, adj));
    EXPECT_EQ(0, adj.size());

    // cloud empty
    cld_->clear();

    Vertices v;
    v.vertices.push_back(uint32_t(1));
    v.vertices.push_back(uint32_t(2));
    v.vertices.push_back(uint32_t(3));
    tri_.push_back(v);

    EXPECT_THROW(TriangulatedCloudToAdjacency(cld_, tri_, adj), ExceptionKeyError);

    // triangles empty
    cld_->push_back(PointXYZ(1.f, 2.f, 3.f));

    tri_.clear();
    EXPECT_NO_THROW(TriangulatedCloudToAdjacency(cld_, tri_, adj));
    EXPECT_EQ(1, adj.size()[0]);
    EXPECT_EQ(1, adj.size()[1]);
}

TEST_F(AdjacencySparseTest, Dims)
{
    for(int i=1; i<=11; i++) {

        CloudXYZPtr cld(new CloudXYZ);
        for(int j=0; j<i; j++) {

            cld->push_back(PointXYZ(1.f, 1.f, 1.f));
        }

        Triangles tri;

        {
            Mat1f adj;
            EXPECT_NO_THROW(TriangulatedCloudToAdjacency(cld, tri, adj));
            EXPECT_MAT_DIMS_EQ(adj, Size2i(i, i))
                    << "Unexpected dimensions for adjacency matrix from triangulated point cloud.";
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
            Mat1f adj;
            EXPECT_NO_THROW(TriangulatedCloudToAdjacency(cld, tri, adj));
            EXPECT_MAT_DIMS_EQ(adj, Size2i(i, i))
                    << "Unexpected dimensions for adjacency matrix from triangulated point cloud."
                    << " After adding triangles.";
        }
    }
}

TEST_F(AdjacencySparseTest, AdjacencyMatValues)
{
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

    Mat1f adj;
    TriangulatedCloudToAdjacency(cld_, tri_, adj);
    EXPECT_MAT_DIMS_EQ(adj, Size2i(NB_VERTICES, NB_VERTICES))
            << "Unexpected dimensions for adjacency matrix from triangulated point cloud.";

    // check adjacency matrix is symmetric
    EXPECT_MAT_EQ(adj, adj.t()) << "Adjacency matrix is not symmetric.";

    // check edges are all positive
    for(int i=0; i<NB_VERTICES; i++) {

        for(int j=0; j<NB_VERTICES; j++) {

            EXPECT_GE(adj(i, j), 0) << "Edge weights must be >=0.";
        }
    }

    // check edge values
    for(int i=0; i<NB_VERTICES; i++) {

        PointXYZ pi = cld_->at(i);

        for(int j=0; j<NB_VERTICES; j++) {

            if(i==j) {
                EXPECT_FLOAT_EQ(0.f, adj(i, j)) << "Self connection found for vertex i=" << i;
            }
            else if(adj(i, j) == 0.f) {

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

                EXPECT_FLOAT_EQ(sqrt(diff_sq.x+diff_sq.y+diff_sq.z), adj(i, j))
                        << "Unexpected edge value.";
            }
        }
    }
}

} // annonymous namespace for tests

#endif // __WITH_PCL

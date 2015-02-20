/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/core/graph/graphattr.h"

#include "gtest/gtest.h"

#include <opencv2/core/core.hpp>

#include "elm/core/debug_utils.h"
#include "elm/core/exception.h"
#include "elm/core/cv/mat_utils_inl.h"
#include "elm/ts/ts.h"

using namespace cv;
using namespace elm;

namespace {

class GraphAttrConstructTest : public ::testing::Test
{
protected:
};

TEST_F(GraphAttrConstructTest, Destruct)
{
    GraphAttr *to = new GraphAttr();
    EXPECT_NO_THROW(delete to);
}

TEST_F(GraphAttrConstructTest, EmptyMap)
{
    Mat1f x;
    GraphAttr to(x, Mat());
    EXPECT_EQ(static_cast<size_t>(0), to.num_vertices());
}

TEST_F(GraphAttrConstructTest, SingleElementMap)
{
    Mat1i x(1, 1, 0);
    EXPECT_THROW(GraphAttr to(x, Mat()), ExceptionBadDims);
    EXPECT_THROW(GraphAttr to(x+1, Mat()), ExceptionBadDims);
}

TEST_F(GraphAttrConstructTest, Max_num_vertices)
{
    for(int n=2; n<11; n++) {

        Mat1i m(n, n);
        for(size_t i=0; i<m.total(); i++) {

            m(i) = i;
        }

        GraphAttr to(m, Mat());
        EXPECT_EQ(static_cast<size_t>(n*n), to.num_vertices());
        EXPECT_EQ(m.total(), to.num_vertices());
    }
}

TEST_F(GraphAttrConstructTest, AdjacencyDense_dims_all_unique)
{
    for(int n=2; n<11; n++) {

        Mat1i m(n, n);
        for(size_t i=0; i<m.total(); i++) {

            m(i) = i;
        }

        GraphAttr to(m, Mat());
        EXPECT_EQ(static_cast<size_t>(n*n), to.num_vertices());
        EXPECT_EQ(m.total(), to.num_vertices());

        Mat1f adj;
        to.AdjacencyMat(adj);
        EXPECT_MAT_DIMS_EQ(adj, Size2i(m.total(), m.total()));
    }
}

/**
 * @brief Test adjacency matrix from graph constructed from
 * map initialized as row matrix.
 *
 * We expect:
 * connections to preceeding and succeeding elements
 */
TEST_F(GraphAttrConstructTest, AdjacencyDense_row_mat)
{
    Mat1i m(1, 5);
    for(size_t i=0; i<m.total(); i++) {

        m(i) = i;
    }

    GraphAttr to(m, Mat());
    EXPECT_EQ(m.total(), to.num_vertices());

    Mat1f adj;
    to.AdjacencyMat(adj);
    EXPECT_MAT_EQ(adj, adj.t()) << "Expecting adj. matrix symmetric around diagonal";

    // verify we have no self-connections
    for(int r=0; r<adj.rows; r++) {
        EXPECT_EQ(0.f, adj(r, r)) << "Unexpected self-connection.";
    }

    // verify that each vertex is only connected to left and right neighbors
    for(int r=0; r<adj.rows; r++) {

        if(r>0 && r<adj.rows-1) {

            EXPECT_FLOAT_EQ(2.f, cv::sum(adj.row(r))[0]) << "Only expecting edges with left and right neighbor";
            EXPECT_FLOAT_EQ(2.f, cv::sum(adj.col(r))[0]) << "Only expecting edges with left and right neighbor";

            // identify neighbors
            EXPECT_FLOAT_EQ(1.f, adj(r-1, r)) << "Missing connection to preceeding neighbor";
            EXPECT_FLOAT_EQ(1.f, adj(r+1, r)) << "Missing connection to succeeding neighbor";
        }
        else {
            // border elements
            EXPECT_FLOAT_EQ(1.f, cv::sum(adj.row(r))[0]) << "Only expecting edges with one neighbor";
            EXPECT_FLOAT_EQ(1.f, cv::sum(adj.col(r))[0]) << "Only expecting edges with one neighbor";

            if(r==0) {
                // first vertex
                EXPECT_FLOAT_EQ(1.f, adj(1)) << "Missing connection to succeeding element";
            }
            else {
                // last vertex
                EXPECT_FLOAT_EQ(1.f, adj(adj.total()-2)) << "Missing connection to preceeding element";
            }
        }
    }
}

TEST_F(GraphAttrConstructTest, AdjacencyDense)
{
    const int ROWS=2;
    const int COLS=3;
    int data[ROWS*COLS] = {1, 1, 2,
                           3, 6, 6};
    Mat1i m = Mat1i(ROWS, COLS, data).clone();

    GraphAttr to(m, Mat());
    const int nb_vertices = static_cast<int>(to.num_vertices());
    EXPECT_EQ(4, nb_vertices);

    Mat1f adj;
    to.AdjacencyMat(adj);
    EXPECT_MAT_DIMS_EQ(adj, Size2i(nb_vertices, nb_vertices));
    EXPECT_MAT_EQ(adj, adj.t()) << "Expecting adj. matrix symmetric around diagonal";

    // verify we have no self-connections
    for(int r=0; r<adj.rows; r++) {
        EXPECT_EQ(0.f, adj(r, r)) << "Unexpected self-connection.";
    }

    // verify individual connections
    EXPECT_FLOAT_EQ(1.f, adj(0, 1));
    EXPECT_FLOAT_EQ(1.f, adj(0, 2));
    EXPECT_FLOAT_EQ(1.f, adj(0, 3));
    EXPECT_FLOAT_EQ(1.f, adj(1, 3));
    EXPECT_FLOAT_EQ(1.f, adj(2, 3));
    EXPECT_FLOAT_EQ(0.f, adj(1, 2));

    // verify vertices vector
    VecF vtx_ids = to.VerticesIds();
    EXPECT_SIZE(nb_vertices, vtx_ids);

    // verify vertex ids and their order
    EXPECT_FLOAT_EQ(1.f, vtx_ids[0]);
    EXPECT_FLOAT_EQ(3.f, vtx_ids[1]);
    EXPECT_FLOAT_EQ(2.f, vtx_ids[2]);
    EXPECT_FLOAT_EQ(6.f, vtx_ids[3]);
}

TEST_F(GraphAttrConstructTest, VerticesIds_size)
{
    for(int n=2; n<11; n++) {

        Mat1i m(n, n);
        for(size_t i=0; i<m.total(); i++) {

            m(i) = i;
        }

        GraphAttr to(m, Mat());
        EXPECT_EQ(static_cast<size_t>(n*n), to.num_vertices());
        EXPECT_EQ(m.total(), to.num_vertices());
        EXPECT_SIZE(n*n, to.VerticesIds());
        EXPECT_SIZE(m.total(), to.VerticesIds());
    }
}

/**
 * @brief verify vertex ids reflect order in adjacency matrix
 */
TEST_F(GraphAttrConstructTest, VerticesIds)
{
    const int ROWS=3;
    const int COLS=3;
    int data[ROWS*COLS] = {1, 1, 2,
                           3, 6, 6,
                           9, 9, 11};
    Mat1i m = Mat1i(ROWS, COLS, data).clone();

    ELM_COUT_VAR(m);

    GraphAttr to(m, Mat());
    const int nb_vertices = static_cast<int>(to.num_vertices());
    EXPECT_EQ(6, nb_vertices);

    Mat1f adj;
    to.AdjacencyMat(adj);

    ELM_COUT_VAR(adj);

    EXPECT_MAT_DIMS_EQ(adj, Size2i(nb_vertices, nb_vertices));
    EXPECT_MAT_EQ(adj, adj.t()) << "Expecting adj. matrix symmetric around diagonal";

    // verify we have no self-connections
    for(int r=0; r<adj.rows; r++) {
        EXPECT_EQ(0.f, adj(r, r)) << "Unexpected self-connection.";
    }

    // verify vertices vector
    VecF vtx_ids = to.VerticesIds();
    EXPECT_SIZE(nb_vertices, vtx_ids);
    for(int i=0; i<adj.rows; i++) {
        std::cout<<vtx_ids[i] << " ";
    }
    std::cout<<std::endl;

    // verify vertex ids and their order
    EXPECT_FLOAT_EQ(1.f, vtx_ids[0]);
    EXPECT_FLOAT_EQ(3.f, vtx_ids[1]);
    EXPECT_FLOAT_EQ(2.f, vtx_ids[2]);
    EXPECT_FLOAT_EQ(6.f, vtx_ids[3]);

    // verify individual connections
    EXPECT_FLOAT_EQ(1.f, adj(0, 1));
    EXPECT_FLOAT_EQ(1.f, adj(0, 2));
    EXPECT_FLOAT_EQ(1.f, adj(0, 3));
    EXPECT_FLOAT_EQ(1.f, adj(1, 3));
    EXPECT_FLOAT_EQ(1.f, adj(2, 3));
    EXPECT_FLOAT_EQ(0.f, adj(1, 2));

    EXPECT_FLOAT_EQ(3.f, cv::sum(adj.row(4))[0]);
    EXPECT_FLOAT_EQ(1.f, adj(1, 4));
    EXPECT_FLOAT_EQ(1.f, adj(3, 4));
    EXPECT_FLOAT_EQ(1.f, adj(5, 4));

    EXPECT_FLOAT_EQ(2.f, cv::sum(adj.row(5))[0]);
    EXPECT_FLOAT_EQ(1.f, adj(3, 5));
    EXPECT_FLOAT_EQ(1.f, adj(4, 5));
}

TEST_F(GraphAttrConstructTest, Mask)
{
    const int ROWS=2;
    const int COLS=3;
    int data[ROWS*COLS] = {1, 1, 2,
                           3, 6, 6};
    Mat1i m = Mat1i(ROWS, COLS, data).clone();

    {
        Mat mask;
        EXPECT_NO_THROW(GraphAttr to(m, mask));
    }
    {
        Mat mask = m >= 0; // all true
        EXPECT_NO_THROW(GraphAttr to(m, mask));
    }
    {
        Mat mask = m < 0; // all false
        EXPECT_NO_THROW(GraphAttr to(m, mask));
    }
    {
        Mat mask = m.t() < 0; // all false
        EXPECT_THROW(GraphAttr to(m, mask), ExceptionBadDims);
    }
    {
        Mat mask = m.t() >= 0; // all true
        EXPECT_THROW(GraphAttr to(m, mask), ExceptionBadDims);
    }
    {
        Mat1b mask(1, 1);
        EXPECT_THROW(GraphAttr to(m, mask), ExceptionBadDims);
    }
    {
        Mat1b mask(ROWS/2, COLS);
        EXPECT_THROW(GraphAttr to(m, mask), ExceptionBadDims);
    }
    {
        Mat1b mask(ROWS/2, COLS, true);
        EXPECT_THROW(GraphAttr to(m, mask), ExceptionBadDims);
    }
    {
        Mat1b mask(ROWS, COLS/2);
        EXPECT_THROW(GraphAttr to(m, mask), ExceptionBadDims);
    }
    {
        Mat1b mask(ROWS, COLS/2, true);
        EXPECT_THROW(GraphAttr to(m, mask), ExceptionBadDims);
    }
}

TEST_F(GraphAttrConstructTest, VerticesIds_masked)
{
    const int ROWS=3;
    const int COLS=3;
    int data[ROWS*COLS] = {1, 1, 2,
                           3, 6, 6,
                           9, 9, 11};
    Mat1i m = Mat1i(ROWS, COLS, data).clone();

    Mat exclude, mask;
    cv::bitwise_or(m < 2, m == 9, exclude);
    cv::bitwise_not(exclude, mask);

    GraphAttr to(m, mask);
    const int nb_vertices = static_cast<int>(to.num_vertices());
    EXPECT_EQ(4, nb_vertices);

    Mat1f adj;
    to.AdjacencyMat(adj);

    EXPECT_MAT_DIMS_EQ(adj, Size2i(nb_vertices, nb_vertices));
    EXPECT_MAT_EQ(adj, adj.t()) << "Expecting adj. matrix symmetric around diagonal";

    // verify we have no self-connections
    for(int r=0; r<adj.rows; r++) {
        EXPECT_EQ(0.f, adj(r, r)) << "Unexpected self-connection.";
    }

    // verify vertices vector
    VecF vtx_ids = to.VerticesIds();
    EXPECT_SIZE(nb_vertices, vtx_ids);

    // verify vertex ids and their order
    EXPECT_FLOAT_EQ(2, vtx_ids[0]);
    EXPECT_FLOAT_EQ(6, vtx_ids[1]);
    EXPECT_FLOAT_EQ(3, vtx_ids[2]);
    EXPECT_FLOAT_EQ(11, vtx_ids[3]);

    // verify individual connections

    EXPECT_FLOAT_EQ(1.f, cv::sum(adj.row(0))[0]);
    EXPECT_FLOAT_EQ(1.f, adj(2, 1));

    EXPECT_FLOAT_EQ(3.f, cv::sum(adj.row(1))[0]);
    EXPECT_FLOAT_EQ(1.f, adj(0, 1));
    EXPECT_FLOAT_EQ(1.f, adj(2, 1));
    EXPECT_FLOAT_EQ(1.f, adj(3, 1));

    EXPECT_FLOAT_EQ(1.f, cv::sum(adj.row(2))[0]);
    EXPECT_FLOAT_EQ(1.f, adj(1, 2));

    EXPECT_FLOAT_EQ(1.f, cv::sum(adj.row(3))[0]);
    EXPECT_FLOAT_EQ(1.f, adj(1, 3));
}

} // annonymous namespace for test cases and fixtures

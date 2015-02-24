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

    GraphAttr to(m, Mat());
    const int nb_vertices = static_cast<int>(to.num_vertices());
    EXPECT_EQ(6, nb_vertices);

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

/**
 * @brief test that vertex ids are unique and no id is repeated
 */
TEST_F(GraphAttrConstructTest, VerticesIds_masked_unique)
{
    const int ROWS=3;
    const int COLS=3;
    float data[ROWS*COLS] = {1.1f, 1.1f, 2.0f,
                             3.0f, 6.0f, 6.0f,
                             9.2f, 9.02f, 11.0f};
    Mat1f m = Mat1f(ROWS, COLS, data).clone();

    Mat1b mask = m >= 2;

    GraphAttr to(m, mask);
    const int nb_vertices = static_cast<int>(to.num_vertices());
    EXPECT_EQ(6, nb_vertices);

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

    for(int i=0; i<nb_vertices; i++) {
        for(int j=i+1; j<nb_vertices; j++) {

            EXPECT_NE(vtx_ids[i], vtx_ids[j]) << "Encountered repeated vertex id.";
        }
    }
}


TEST_F(GraphAttrConstructTest, AddAttributes_empty_graph)
{
    Mat1f x;
    GraphAttr to(x, Mat());
    EXPECT_EQ(static_cast<size_t>(0), to.num_vertices());

    float vtx_id = -10.f;
    while(++vtx_id <= 10.f) {

        EXPECT_THROW(to.addAttributes(++vtx_id, Mat1f()), ExceptionKeyError);

        EXPECT_THROW(to.addAttributes(++vtx_id, Mat1f(2, 3, 1.f)), ExceptionKeyError);
    }
}

TEST_F(GraphAttrConstructTest, AddAttributes_invalid_vtx_id)
{
    const int ROWS=3;
    const int COLS=3;
    int data[ROWS*COLS] = {1, 1, 2,
                           3, 6, 6,
                           9, 9, 11};
    Mat1i m = Mat1i(ROWS, COLS, data).clone();

    GraphAttr to(m, Mat1b());

    ASSERT_GT(to.num_vertices(), static_cast<size_t>(0)) << "this test requires a non-empty graph";

    EXPECT_THROW(to.addAttributes(-1.f, Mat1f()), ExceptionKeyError);
    EXPECT_THROW(to.addAttributes(-1.f, Mat1f(2, 3, 1.f)), ExceptionKeyError);

    EXPECT_THROW(to.addAttributes(0.f, Mat1f()), ExceptionKeyError);
    EXPECT_THROW(to.addAttributes(0.f, Mat1f(2, 3, 1.f)), ExceptionKeyError);

    EXPECT_THROW(to.addAttributes(5.f, Mat1f()), ExceptionKeyError);
    EXPECT_THROW(to.addAttributes(5.f, Mat1f(2, 3, 1.f)), ExceptionKeyError);

    EXPECT_THROW(to.addAttributes(11.2f, Mat1f()), ExceptionKeyError);
    EXPECT_THROW(to.addAttributes(11.2, Mat1f(2, 3, 1.f)), ExceptionKeyError);

    EXPECT_NO_THROW(to.addAttributes(9.f, Mat1f()));
    EXPECT_NO_THROW(to.addAttributes(9.f, Mat1f(2, 3, 1.f)));
}

TEST_F(GraphAttrConstructTest, AddAttributes_invalid_vtx_id_masked)
{
    const int ROWS=3;
    const int COLS=3;
    int data[ROWS*COLS] = {1, 1, 2,
                           3, 6, 6,
                           9, 9, 11};
    Mat1i m = Mat1i(ROWS, COLS, data).clone();

    Mat1b exclude, mask;
    cv::bitwise_or(m < 2, m == 9, exclude);
    cv::bitwise_not(exclude, mask);

    GraphAttr to(m, mask);

    ASSERT_GT(to.num_vertices(), static_cast<size_t>(0)) << "this test requires a non-empty graph";

    EXPECT_THROW(to.addAttributes(-1.f, Mat1f()), ExceptionKeyError);
    EXPECT_THROW(to.addAttributes(-1.f, Mat1f(2, 3, 1.f)), ExceptionKeyError);

    EXPECT_THROW(to.addAttributes(0.f, Mat1f()), ExceptionKeyError);
    EXPECT_THROW(to.addAttributes(0.f, Mat1f(2, 3, 1.f)), ExceptionKeyError);

    EXPECT_THROW(to.addAttributes(5.f, Mat1f()), ExceptionKeyError);
    EXPECT_THROW(to.addAttributes(5.f, Mat1f(2, 3, 1.f)), ExceptionKeyError);

    EXPECT_THROW(to.addAttributes(11.2f, Mat1f()), ExceptionKeyError);
    EXPECT_THROW(to.addAttributes(11.2, Mat1f(2, 3, 1.f)), ExceptionKeyError);

    EXPECT_THROW(to.addAttributes(9.f, Mat1f()), ExceptionKeyError);
    EXPECT_THROW(to.addAttributes(9.f, Mat1f(2, 3, 1.f)), ExceptionKeyError);
}

TEST_F(GraphAttrConstructTest, GetAttributes_empty_graph)
{
    Mat1f x;
    GraphAttr to(x, Mat());
    EXPECT_EQ(static_cast<size_t>(0), to.num_vertices());

    float vtx_id = -10.f;
    while(++vtx_id <= 10.f) {

        EXPECT_THROW(to.getAttributes(++vtx_id), ExceptionKeyError);
    }
}

TEST_F(GraphAttrConstructTest, GetAttributes_invalid_vtx_id)
{
    const int ROWS=3;
    const int COLS=3;
    int data[ROWS*COLS] = {1, 1, 2,
                           3, 6, 6,
                           9, 9, 11};
    Mat1i m = Mat1i(ROWS, COLS, data).clone();

    GraphAttr to(m, Mat1b());

    ASSERT_GT(to.num_vertices(), static_cast<size_t>(0)) << "this test requires a non-empty graph";

    EXPECT_THROW(to.getAttributes(-1.f), ExceptionKeyError);
    EXPECT_THROW(to.getAttributes(0.f), ExceptionKeyError);
    EXPECT_THROW(to.getAttributes(12.f), ExceptionKeyError);

    EXPECT_THROW(to.getAttributes(4.f), ExceptionKeyError);
    EXPECT_THROW(to.getAttributes(5.f), ExceptionKeyError);

    for(size_t i=0; i<m.total(); i++) {

        EXPECT_NO_THROW(to.getAttributes(m(i)));
        EXPECT_THROW(to.getAttributes(m(i)+0.2f), ExceptionKeyError);
        EXPECT_THROW(to.getAttributes(m(i)-0.2f), ExceptionKeyError);
        EXPECT_THROW(to.getAttributes(m(i)+0.8f), ExceptionKeyError);
        EXPECT_THROW(to.getAttributes(m(i)-0.8f), ExceptionKeyError);
    }
}

TEST_F(GraphAttrConstructTest, GetAttributes_invalid_vtx_id_masked)
{
    const int ROWS=3;
    const int COLS=3;
    int data[ROWS*COLS] = {1, 1, 2,
                           3, 6, 6,
                           9, 9, 11};
    Mat1i m = Mat1i(ROWS, COLS, data).clone();

    Mat1b exclude, mask;
    cv::bitwise_or(m < 2, m == 9, exclude);
    cv::bitwise_not(exclude, mask);

    GraphAttr to(m, mask);

    ASSERT_GT(to.num_vertices(), static_cast<size_t>(0)) << "this test requires a non-empty graph";

    EXPECT_THROW(to.getAttributes(-1.f), ExceptionKeyError);

    EXPECT_THROW(to.getAttributes(0.f), ExceptionKeyError);

    EXPECT_THROW(to.getAttributes(5.f), ExceptionKeyError);

    EXPECT_THROW(to.getAttributes(11.2f), ExceptionKeyError);

    EXPECT_THROW(to.getAttributes(9.f), ExceptionKeyError);

    for(size_t i=0; i<m.total(); i++) {

        if(mask(i)) {
            EXPECT_NO_THROW(to.getAttributes(m(i)));
            EXPECT_THROW(to.getAttributes(m(i)+0.2f), ExceptionKeyError);
            EXPECT_THROW(to.getAttributes(m(i)-0.2f), ExceptionKeyError);
            EXPECT_THROW(to.getAttributes(m(i)+0.8f), ExceptionKeyError);
            EXPECT_THROW(to.getAttributes(m(i)-0.8f), ExceptionKeyError);
        }
        else {
            EXPECT_THROW(to.getAttributes(m(i)), ExceptionKeyError);
        }
    }
}

TEST_F(GraphAttrConstructTest, Add_and_getAttributes_masked)
{
    const int ROWS=3;
    const int COLS=3;
    float data[ROWS*COLS] = {1, 1, 2.2,
                             3, 6, 6,
                             9, 9.5, 11};
    Mat1f m = Mat1f(ROWS, COLS, data).clone();

    Mat1b exclude, mask;
    cv::bitwise_or(m < 2, m == 9, exclude);
    cv::bitwise_not(exclude, mask);

    GraphAttr to(m, mask);

    ASSERT_GT(to.num_vertices(), static_cast<size_t>(0)) << "this test requires a non-empty graph";

    VecF vtx_ids = to.VerticesIds();

    // populate vertex attributes with arbitrary values
    for(size_t i=0; i<vtx_ids.size(); i++) {

        to.addAttributes(vtx_ids[i], Mat1f(1, static_cast<int>(vtx_ids[i])+1, vtx_ids[i]*2.f));
    }

    for(size_t i=0; i<vtx_ids.size(); i++) {

        EXPECT_MAT_EQ(to.getAttributes(vtx_ids[i]), Mat1f(1, static_cast<int>(vtx_ids[i])+1, vtx_ids[i]*2.f));
    }
}

/**
 * @brief toy function that returns an image with non-masked pixels zero'ed out
 * @param img
 * @param mask
 * @return masked_img
 */
Mat1f func_masked_img(const cv::Mat1f &img, const cv::Mat1b &mask) {

    return img.clone().setTo(0.f, mask == 0);
}

TEST_F(GraphAttrConstructTest, ApplyVerticesToMap_masked_img)
{
    const int ROWS=3;
    const int COLS=3;
    float data[ROWS*COLS] = {1, 1, 2.2,
                             3, 6, 6,
                             9, 9.5, 11};
    Mat1f m = Mat1f(ROWS, COLS, data).clone();

    Mat1b exclude, mask;
    cv::bitwise_or(m < 2, m == 9, exclude);
    cv::bitwise_not(exclude, mask);

    GraphAttr to(m, mask);

    ASSERT_GT(to.num_vertices(), static_cast<size_t>(0)) << "this test requires a non-empty graph";

    VecF vtx_ids = to.VerticesIds();

    VecMat1f results = to.applyVerticesToMap(func_masked_img);

    EXPECT_EQ(vtx_ids.size(), results.size()) << "Expecting exactly 1 item per vertex.";

    for(size_t i=0; i<vtx_ids.size(); i++) {

        float vtx_id = vtx_ids[i];

        EXPECT_MAT_EQ(m.clone().setTo(0.f, m != vtx_id), results[i]);

    }
}

TEST_F(GraphAttrConstructTest, ApplyVertexToMap_masked_img)
{
    const int ROWS=3;
    const int COLS=3;
    float data[ROWS*COLS] = {1, 1, 2.2,
                             3, 6, 6,
                             9, 9.5, 11};
    Mat1f m = Mat1f(ROWS, COLS, data).clone();

    Mat1b exclude, mask;
    cv::bitwise_or(m < 2, m == 9, exclude);
    cv::bitwise_not(exclude, mask);

    GraphAttr to(m, mask);

    ASSERT_GT(to.num_vertices(), static_cast<size_t>(0)) << "this test requires a non-empty graph";

    VecF vtx_ids = to.VerticesIds();

    for(size_t i=0; i<vtx_ids.size(); i++) {

        float vtx_id = vtx_ids[i];

        Mat1f result = to.applyVertexToMap(vtx_id, func_masked_img);

        EXPECT_MAT_EQ(m.clone().setTo(0.f, m != vtx_id), result);
    }
}

TEST_F(GraphAttrConstructTest, ApplyVertexToMap_masked_img_invalid_vtx_id)
{
    const int ROWS=3;
    const int COLS=3;
    float data[ROWS*COLS] = {1, 1, 2.2,
                             3, 6, 6,
                             9, 9.5, 11};
    Mat1f m = Mat1f(ROWS, COLS, data).clone();

    Mat1b exclude, mask;
    cv::bitwise_or(m < 2, m == 9, exclude);
    cv::bitwise_not(exclude, mask);

    GraphAttr to(m, mask);

    ASSERT_GT(to.num_vertices(), static_cast<size_t>(0)) << "this test requires a non-empty graph";

    EXPECT_THROW(to.applyVertexToMap(-1.f, func_masked_img), ExceptionKeyError);

    EXPECT_THROW(to.applyVertexToMap(0.f, func_masked_img), ExceptionKeyError);

    EXPECT_THROW(to.applyVertexToMap(5.f, func_masked_img), ExceptionKeyError);

    EXPECT_THROW(to.applyVertexToMap(11.2f, func_masked_img), ExceptionKeyError);

    EXPECT_THROW(to.applyVertexToMap(9.f, func_masked_img), ExceptionKeyError);

    for(size_t i=0; i<m.total(); i++) {

        if(mask(i)) {
            EXPECT_NO_THROW(to.applyVertexToMap(m(i), func_masked_img));
            EXPECT_THROW(to.applyVertexToMap(m(i)+0.2f, func_masked_img), ExceptionKeyError);
            EXPECT_THROW(to.applyVertexToMap(m(i)-0.2f, func_masked_img), ExceptionKeyError);
            EXPECT_THROW(to.applyVertexToMap(m(i)+0.7f, func_masked_img), ExceptionKeyError);
            EXPECT_THROW(to.applyVertexToMap(m(i)-0.7f, func_masked_img), ExceptionKeyError);
        }
        else {
            EXPECT_THROW(to.applyVertexToMap(m(i), func_masked_img), ExceptionKeyError);
        }
    }
}

/**
 * @brief testing GraphAttr methods after sucessful construction
 * form masked image map
 */
class GraphAttrMaskedTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        const int ROWS=3;
        const int COLS=3;
        float data[ROWS*COLS] = {1.f, 7.0f, 2.2f,
                                 3.f, 6.0f, 6.0f,
                                 9.f, 9.5f, 11.f};
        map_ = Mat1f(ROWS, COLS, data).clone();

        Mat1b exclude;
        cv::bitwise_or(map_ < 2.f, map_ == 9.f, exclude);
        cv::bitwise_not(exclude, mask_);

        to_ = GraphAttr(map_, mask_);
    }

    // members
    GraphAttr to_;  ///< test object
    Mat1f map_;
    Mat1b mask_;
};

TEST_F(GraphAttrMaskedTest, ApplyVertexToMap_masked_img_invalid_vtx_id)
{
    ASSERT_GT(to_.num_vertices(), static_cast<size_t>(0)) << "this test requires a non-empty graph";

    EXPECT_THROW(to_.applyVertexToMap(-1.f, func_masked_img), ExceptionKeyError);

    EXPECT_THROW(to_.applyVertexToMap(0.f, func_masked_img), ExceptionKeyError);

    EXPECT_THROW(to_.applyVertexToMap(5.f, func_masked_img), ExceptionKeyError);

    EXPECT_THROW(to_.applyVertexToMap(11.2f, func_masked_img), ExceptionKeyError);

    EXPECT_THROW(to_.applyVertexToMap(9.f, func_masked_img), ExceptionKeyError);

    for(size_t i=0; i<map_.total(); i++) {

        if(mask_(i)) {
            EXPECT_NO_THROW(to_.applyVertexToMap(map_(i), func_masked_img));
            EXPECT_THROW(to_.applyVertexToMap(map_(i)+0.2f, func_masked_img), ExceptionKeyError);
            EXPECT_THROW(to_.applyVertexToMap(map_(i)-0.2f, func_masked_img), ExceptionKeyError);
            EXPECT_THROW(to_.applyVertexToMap(map_(i)+0.7f, func_masked_img), ExceptionKeyError);
            EXPECT_THROW(to_.applyVertexToMap(map_(i)-0.7f, func_masked_img), ExceptionKeyError);
        }
        else {
            EXPECT_THROW(to_.applyVertexToMap(map_(i), func_masked_img), ExceptionKeyError);
        }
    }
}

TEST_F(GraphAttrMaskedTest, RemoveEdges_invalid)
{
    EXPECT_THROW(to_.removeEdges(2.2f, 9.0f), ExceptionKeyError);
    EXPECT_THROW(to_.removeEdges(9.0f, 2.2f), ExceptionKeyError);
}

TEST_F(GraphAttrMaskedTest, VertexIndex)
{
    VecF vtx_ids = to_.VerticesIds();
    for(size_t i=0; i<vtx_ids.size(); i++) {

        EXPECT_EQ(static_cast<int>(i), to_.VertexIndex(vtx_ids[i]));

        EXPECT_THROW(to_.VertexIndex(vtx_ids[i]+100.f), ExceptionKeyError);
    }
}

TEST_F(GraphAttrMaskedTest, VertexIndex_graph_without_vertices)
{
    GraphAttr to;
    VecF vtx_ids = to.VerticesIds();

    ASSERT_EQ(size_t(0), vtx_ids.size()) << "Expecting empty graph, without vertices";

    EXPECT_THROW(to.VertexIndex(0.f), ExceptionKeyError);
    EXPECT_THROW(to.VertexIndex(2.2f), ExceptionKeyError);
    EXPECT_THROW(to.VertexIndex(6.f), ExceptionKeyError);
    EXPECT_THROW(to.VertexIndex(9.f), ExceptionKeyError);
}

/**
 * @brief Remove edge between two vertices that weren't connected
 * in the first place.
 */
TEST_F(GraphAttrMaskedTest, RemoveEdges_no_edge)
{
    Mat1f adj0;
    to_.AdjacencyMat(adj0);

    float u = 2.2f;
    float v = 11.0f;
    to_.removeEdges(u, v); // no edge

    Mat1f adj;
    to_.AdjacencyMat(adj);

    ASSERT_EQ(0.f, adj(to_.VertexIndex(u), to_.VertexIndex(v)))
            << "test requires two vertices that are not connected. to one another.";
    ASSERT_EQ(0.f, adj(to_.VertexIndex(v), to_.VertexIndex(u)))
            << "test requires two vertices that are not connected. to one another.";

    EXPECT_MAT_EQ(adj0, adj) << "Adjacency matrix changed after redge rmeoval.";
}

TEST_F(GraphAttrMaskedTest, RemoveEdges)
{
    float u = 2.2f;
    float v = 6.0f;

    int u_idx = to_.VertexIndex(u);
    int v_idx = to_.VertexIndex(v);

    Mat1f adj0;
    to_.AdjacencyMat(adj0);

    ASSERT_GT(adj0(u_idx, v_idx), 0.f) << "test assumes edge between vertices exists.";
    ASSERT_GT(adj0(v_idx, u_idx), 0.f) << "test assumes edge between vertices exists.";

    to_.removeEdges(u, v);

    Mat1f adj;
    to_.AdjacencyMat(adj);

    EXPECT_EQ(0.f, adj(u_idx, v_idx));
    EXPECT_EQ(0.f, adj(v_idx, u_idx));
}

/**
 * @brief Test that edge contraction/merging decreases number of vertices
 */
TEST_F(GraphAttrMaskedTest, ContractEdges_new_dims)
{
    const int NB_VERTICES = static_cast<int>(to_.num_vertices());

    ASSERT_GT(NB_VERTICES, 1) << "this test requires a graph with at least 2 vertices.";

    for(int i=1; i<NB_VERTICES; i++) {

        VecF vtx_ids = to_.VerticesIds();
        float u = vtx_ids[0];
        float v = vtx_ids[1];

        Mat1f adj_pre;
        to_.AdjacencyMat(adj_pre);

        to_.contractEdges(u, v);

        EXPECT_EQ(size_t(NB_VERTICES-i), to_.num_vertices()) << "no. of vertices not decreasing.";
        EXPECT_SIZE(size_t(NB_VERTICES-i), to_.VerticesIds()) << "no. of vertices not decreasing.";

        Mat1f adj;
        to_.AdjacencyMat(adj);

        EXPECT_MAT_DIMS_EQ(adj, Size2i(adj_pre.cols-1, adj_pre.rows-1))
                << "Expecting adj. matrix to shrink by 1 row and 1 column.";
    }
}

/**
 * @brief Verify which vertex got removed after merge
 */
TEST_F(GraphAttrMaskedTest, ContractEdges_vtx_removed)
{
    ASSERT_GT(static_cast<int>(to_.num_vertices()), 1) << "this test requires a graph with at least 2 vertices.";

    while(static_cast<int>(to_.num_vertices()) > 1) {

        VecF vtx_ids = to_.VerticesIds();
        float u = vtx_ids[1];
        float v = vtx_ids[0];

        Mat1f adj_pre;
        to_.AdjacencyMat(adj_pre);

        to_.contractEdges(u, v);

        EXPECT_THROW(to_.VertexIndex(u), ExceptionKeyError);
        EXPECT_EQ(0, to_.VertexIndex(v));
    }
}

TEST_F(GraphAttrMaskedTest, ContractEdges_merge_neighbors)
{
    float u = 6.0f;
    float v = 9.5f;

    int u_idx = to_.VertexIndex(u);
    int v_idx = to_.VertexIndex(v);

    Mat1f adj0;
    to_.AdjacencyMat(adj0);

    to_.contractEdges(u, v);

    // perform OR of adj. row and column for both vertices
    // and enforce all-zero diagonal
    Mat1f adj_or = adj0.clone();
    cv::bitwise_or(adj_or.row(u_idx), adj_or.row(v_idx),
                   adj_or.row(v_idx));
    cv::bitwise_or(adj_or.col(u_idx), adj_or.col(v_idx),
                   adj_or.col(v_idx));
    for(int r=0; r<adj_or.rows; r++) {
        adj_or(r, r) = 0.f;
    }

    Mat1f adj;
    to_.AdjacencyMat(adj);

    // iterate through both and compare connectivity after merge.
    // skip vertex that was omitted by merge
    int r_m = -1;
    for(int r=0; r<adj.rows; r++) {

        if(r!=u_idx) {
            r_m++;
        }
        int c_m = -1;
        for(int c=0; c<adj.cols; c++) {

            if(c!=u_idx) {
                c_m++;
            }

            if(r!=u_idx && c!=u_idx) {

                EXPECT_FLOAT_EQ(adj_or(r, c), adj(r_m, c_m));
            }
        }
    }
}

} // annonymous namespace for test cases and fixtures

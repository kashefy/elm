/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/core/graph/graphattr.h"

#include "gtest/gtest.h"

#include <set>

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
    Mat1i x;
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
    VecI vtx_ids = to.VerticesIds();
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
    VecI vtx_ids = to.VerticesIds();
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
    VecI vtx_ids = to.VerticesIds();
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
    int data[ROWS*COLS] = {1, 1, 2,
                           3, 6, 6,
                           9, 9, 11};
    Mat1i m = Mat1i(ROWS, COLS, data).clone();

    Mat1b mask = m >= 2;

    GraphAttr to(m, mask);
    const int nb_vertices = static_cast<int>(to.num_vertices());
    EXPECT_EQ(5, nb_vertices);

    Mat1f adj;
    to.AdjacencyMat(adj);

    EXPECT_MAT_DIMS_EQ(adj, Size2i(nb_vertices, nb_vertices));
    EXPECT_MAT_EQ(adj, adj.t()) << "Expecting adj. matrix symmetric around diagonal";

    // verify we have no self-connections
    for(int r=0; r<adj.rows; r++) {
        EXPECT_EQ(0.f, adj(r, r)) << "Unexpected self-connection.";
    }

    // verify vertices vector
    VecI vtx_ids = to.VerticesIds();
    EXPECT_SIZE(nb_vertices, vtx_ids);

    for(int i=0; i<nb_vertices; i++) {
        for(int j=i+1; j<nb_vertices; j++) {

            EXPECT_NE(vtx_ids[i], vtx_ids[j]) << "Encountered repeated vertex id.";
        }
    }
}


TEST_F(GraphAttrConstructTest, AddAttributes_empty_graph)
{
    Mat1i x;
    GraphAttr to(x, Mat());
    EXPECT_EQ(static_cast<size_t>(0), to.num_vertices());

    int vtx_id = -1;
    while(++vtx_id <= 10) {

        EXPECT_THROW(to.addAttributes(vtx_id, Mat1f()), ExceptionKeyError);

        EXPECT_THROW(to.addAttributes(vtx_id, Mat1f(2, 3, 1.f)), ExceptionKeyError);
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

    EXPECT_THROW(to.addAttributes(0, Mat1f()), ExceptionKeyError);
    EXPECT_THROW(to.addAttributes(0, Mat1f(2, 3, 1.f)), ExceptionKeyError);

    EXPECT_THROW(to.addAttributes(5, Mat1f()), ExceptionKeyError);
    EXPECT_THROW(to.addAttributes(5, Mat1f(2, 3, 1.f)), ExceptionKeyError);

    EXPECT_THROW(to.addAttributes(12, Mat1f()), ExceptionKeyError);
    EXPECT_THROW(to.addAttributes(12, Mat1f(2, 3, 1.f)), ExceptionKeyError);

    EXPECT_NO_THROW(to.addAttributes(9, Mat1f()));
    EXPECT_NO_THROW(to.addAttributes(9, Mat1f(2, 3, 1.f)));
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

    EXPECT_THROW(to.addAttributes(0, Mat1f()), ExceptionKeyError);
    EXPECT_THROW(to.addAttributes(0, Mat1f(2, 3, 1.f)), ExceptionKeyError);

    EXPECT_THROW(to.addAttributes(5, Mat1f()), ExceptionKeyError);
    EXPECT_THROW(to.addAttributes(5, Mat1f(2, 3, 1.f)), ExceptionKeyError);

    EXPECT_THROW(to.addAttributes(12, Mat1f()), ExceptionKeyError);
    EXPECT_THROW(to.addAttributes(12, Mat1f(2, 3, 1.f)), ExceptionKeyError);

    EXPECT_THROW(to.addAttributes(9, Mat1f()), ExceptionKeyError);
    EXPECT_THROW(to.addAttributes(9, Mat1f(2, 3, 1.f)), ExceptionKeyError);
}

TEST_F(GraphAttrConstructTest, GetAttributes_empty_graph)
{
    Mat1f x;
    GraphAttr to(x, Mat());
    EXPECT_EQ(static_cast<size_t>(0), to.num_vertices());

    float vtx_id = -1;
    while(++vtx_id <= 10) {

        EXPECT_THROW(to.getAttributes(vtx_id), ExceptionKeyError);
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

    EXPECT_THROW(to.getAttributes(-1), ExceptionKeyError);
    EXPECT_THROW(to.getAttributes(0), ExceptionKeyError);
    EXPECT_THROW(to.getAttributes(12), ExceptionKeyError);

    EXPECT_THROW(to.getAttributes(4), ExceptionKeyError);
    EXPECT_THROW(to.getAttributes(5), ExceptionKeyError);

    for(size_t i=0; i<m.total(); i++) {

        EXPECT_NO_THROW(to.getAttributes(m(i)));
        EXPECT_THROW(to.getAttributes(m(i)+100), ExceptionKeyError);
        EXPECT_THROW(to.getAttributes(m(i)-100), ExceptionKeyError);
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

    EXPECT_THROW(to.getAttributes(-1), ExceptionKeyError);

    EXPECT_THROW(to.getAttributes(0), ExceptionKeyError);

    EXPECT_THROW(to.getAttributes(5), ExceptionKeyError);

    EXPECT_THROW(to.getAttributes(12), ExceptionKeyError);

    EXPECT_THROW(to.getAttributes(9), ExceptionKeyError);

    for(size_t i=0; i<m.total(); i++) {

        if(mask(i)) {
            EXPECT_NO_THROW(to.getAttributes(m(i)));
            EXPECT_THROW(to.getAttributes(m(i)+20), ExceptionKeyError);
            EXPECT_THROW(to.getAttributes(m(i)-20), ExceptionKeyError);
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
    int data[ROWS*COLS] = {1, 1, 2,
                           3, 6, 6,
                           9, 9, 11};
    Mat1i m = Mat1i(ROWS, COLS, data).clone();

    Mat1b exclude, mask;
    cv::bitwise_or(m < 2, m == 9, exclude);
    cv::bitwise_not(exclude, mask);

    GraphAttr to(m, mask);

    ASSERT_GT(to.num_vertices(), static_cast<size_t>(0)) << "this test requires a non-empty graph";

    VecI vtx_ids = to.VerticesIds();

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

    return img.clone().setTo(0, mask == 0);
}

TEST_F(GraphAttrConstructTest, ApplyVerticesToMap_masked_img)
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

    VecI vtx_ids = to.VerticesIds();

    VecMat1f results = to.applyVerticesToMap(func_masked_img);

    EXPECT_EQ(vtx_ids.size(), results.size()) << "Expecting exactly 1 item per vertex.";

    for(size_t i=0; i<vtx_ids.size(); i++) {

        int vtx_id = vtx_ids[i];

        EXPECT_MAT_EQ(Mat1f(m.clone().setTo(0, m != vtx_id)), results[i]);

    }
}

TEST_F(GraphAttrConstructTest, ApplyVertexToMap_masked_img)
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

    VecI vtx_ids = to.VerticesIds();

    for(size_t i=0; i<vtx_ids.size(); i++) {

        float vtx_id = vtx_ids[i];

        Mat1f result = to.applyVertexToMap(vtx_id, func_masked_img);

        EXPECT_MAT_EQ(Mat1f(m.clone().setTo(0, m != vtx_id)), result);
    }
}

TEST_F(GraphAttrConstructTest, ApplyVertexToMap_masked_img_invalid_vtx_id)
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

    EXPECT_THROW(to.applyVertexToMap(-1, func_masked_img), ExceptionKeyError);

    EXPECT_THROW(to.applyVertexToMap(0, func_masked_img), ExceptionKeyError);

    EXPECT_THROW(to.applyVertexToMap(5, func_masked_img), ExceptionKeyError);

    EXPECT_THROW(to.applyVertexToMap(12, func_masked_img), ExceptionKeyError);

    EXPECT_THROW(to.applyVertexToMap(9, func_masked_img), ExceptionKeyError);

    for(size_t i=0; i<m.total(); i++) {

        if(mask(i)) {
            EXPECT_NO_THROW(to.applyVertexToMap(m(i), func_masked_img));
            EXPECT_THROW(to.applyVertexToMap(m(i)+20, func_masked_img), ExceptionKeyError);
            EXPECT_THROW(to.applyVertexToMap(m(i)-20, func_masked_img), ExceptionKeyError);
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
        int data[ROWS*COLS] = {1, 7, 2,
                               3, 6, 6,
                               9, 9, 11};
        map_ = Mat1i(ROWS, COLS, data).clone();

        Mat1b exclude;
        cv::bitwise_or(map_ < 2, map_ == 9, exclude);
        cv::bitwise_not(exclude, mask_);

        to_ = GraphAttr(map_, mask_);
    }

    // members
    GraphAttr to_;  ///< test object
    Mat1i map_;
    Mat1b mask_;
};

TEST_F(GraphAttrMaskedTest, ApplyVertexToMap_masked_img_invalid_vtx_id)
{
    ASSERT_GT(to_.num_vertices(), static_cast<size_t>(0)) << "this test requires a non-empty graph";

    EXPECT_THROW(to_.applyVertexToMap(-1, func_masked_img), ExceptionKeyError);

    EXPECT_THROW(to_.applyVertexToMap(0, func_masked_img), ExceptionKeyError);

    EXPECT_THROW(to_.applyVertexToMap(5, func_masked_img), ExceptionKeyError);

    EXPECT_THROW(to_.applyVertexToMap(12, func_masked_img), ExceptionKeyError);

    EXPECT_THROW(to_.applyVertexToMap(9, func_masked_img), ExceptionKeyError);

    for(size_t i=0; i<map_.total(); i++) {

        if(mask_(i)) {
            EXPECT_NO_THROW(to_.applyVertexToMap(map_(i), func_masked_img));
            EXPECT_THROW(to_.applyVertexToMap(map_(i)+20, func_masked_img), ExceptionKeyError);
            EXPECT_THROW(to_.applyVertexToMap(map_(i)-20, func_masked_img), ExceptionKeyError);
        }
        else {
            EXPECT_THROW(to_.applyVertexToMap(map_(i), func_masked_img), ExceptionKeyError);
        }
    }
}

class DummyWithStaticMethod
{
public:
    static Mat1f func_masked_img(const cv::Mat1f &img, const cv::Mat1b &mask) {

        return img.clone().setTo(0.f, mask == 0);
    }
};

TEST_F(GraphAttrMaskedTest, ApplyVertexToMap_masked_img_invalid_vtx_id_static_method)
{
    ASSERT_GT(to_.num_vertices(), static_cast<size_t>(0)) << "this test requires a non-empty graph";

    EXPECT_THROW(to_.applyVertexToMap(-1, DummyWithStaticMethod::func_masked_img), ExceptionKeyError);

    EXPECT_THROW(to_.applyVertexToMap(0, DummyWithStaticMethod::func_masked_img), ExceptionKeyError);

    EXPECT_THROW(to_.applyVertexToMap(5, DummyWithStaticMethod::func_masked_img), ExceptionKeyError);

    EXPECT_THROW(to_.applyVertexToMap(12, DummyWithStaticMethod::func_masked_img), ExceptionKeyError);

    EXPECT_THROW(to_.applyVertexToMap(9, DummyWithStaticMethod::func_masked_img), ExceptionKeyError);

    for(size_t i=0; i<map_.total(); i++) {

        if(mask_(i)) {
            EXPECT_NO_THROW(to_.applyVertexToMap(map_(i), DummyWithStaticMethod::func_masked_img));
            EXPECT_THROW(to_.applyVertexToMap(map_(i)+20, DummyWithStaticMethod::func_masked_img), ExceptionKeyError);
            EXPECT_THROW(to_.applyVertexToMap(map_(i)-20, DummyWithStaticMethod::func_masked_img), ExceptionKeyError);
        }
        else {
            EXPECT_THROW(to_.applyVertexToMap(map_(i), DummyWithStaticMethod::func_masked_img), ExceptionKeyError);
        }
    }
}


TEST_F(GraphAttrConstructTest, ApplyVertexToMap_masked_img_static_method)
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

    VecI vtx_ids = to.VerticesIds();

    for(size_t i=0; i<vtx_ids.size(); i++) {

        int vtx_id = vtx_ids[i];

        Mat1f result = to.applyVertexToMap(vtx_id, DummyWithStaticMethod::func_masked_img);

        EXPECT_MAT_EQ(Mat1f(m.clone().setTo(0.f, m != vtx_id)), result);
    }
}

TEST_F(GraphAttrConstructTest, ApplyVerticesToMap_masked_img_static_method)
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

    VecI vtx_ids = to.VerticesIds();

    VecMat1f results = to.applyVerticesToMap(DummyWithStaticMethod::func_masked_img);

    EXPECT_EQ(vtx_ids.size(), results.size()) << "Expecting exactly 1 item per vertex.";

    for(size_t i=0; i<vtx_ids.size(); i++) {

        int vtx_id = vtx_ids[i];
        EXPECT_MAT_EQ(Mat1f(m.clone().setTo(0.f, m != vtx_id)), results[i]);
    }
}

TEST_F(GraphAttrMaskedTest, RemoveEdges_invalid)
{
    EXPECT_THROW(to_.removeEdges(20, 9), ExceptionKeyError);
    EXPECT_THROW(to_.removeEdges(9, 20), ExceptionKeyError);
}

TEST_F(GraphAttrMaskedTest, VertexIndex)
{
    VecI vtx_ids = to_.VerticesIds();
    for(size_t i=0; i<vtx_ids.size(); i++) {

        EXPECT_EQ(static_cast<int>(i), to_.VertexIndex(vtx_ids[i]));

        EXPECT_THROW(to_.VertexIndex(vtx_ids[i]+100), ExceptionKeyError);
    }
}

TEST_F(GraphAttrMaskedTest, VertexIndex_graph_without_vertices)
{
    GraphAttr to;
    VecI vtx_ids = to.VerticesIds();

    ASSERT_EQ(size_t(0), vtx_ids.size()) << "Expecting empty graph, without vertices";

    EXPECT_THROW(to.VertexIndex(0), ExceptionKeyError);
    EXPECT_THROW(to.VertexIndex(22), ExceptionKeyError);
    EXPECT_THROW(to.VertexIndex(6), ExceptionKeyError);
    EXPECT_THROW(to.VertexIndex(9), ExceptionKeyError);
}

/**
 * @brief Remove edge between two vertices that weren't connected
 * in the first place.
 */
TEST_F(GraphAttrMaskedTest, RemoveEdges_no_edge)
{
    Mat1f adj0;
    to_.AdjacencyMat(adj0);

    int u = 2;
    int v = 11;
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
    int u = 2;
    int v = 6;

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

        VecI vtx_ids = to_.VerticesIds();
        //ELM_COUT_VAR(elm::to_string(vtx_ids));
        int u = vtx_ids[0];
        int v = vtx_ids[1];

        Mat1f adj_pre;
        to_.AdjacencyMat(adj_pre);

        to_.contractEdges(u, v);
        //ELM_COUT_VAR(elm::to_string(to_.VerticesIds()));

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

        VecI vtx_ids = to_.VerticesIds();
        int u = vtx_ids[1];
        int v = vtx_ids[0];

        to_.contractEdges(u, v);

        EXPECT_THROW(to_.VertexIndex(u), ExceptionKeyError);
        EXPECT_EQ(0, to_.VertexIndex(v));
    }
}

TEST_F(GraphAttrMaskedTest, ContractEdges_merge_neighbors)
{
    int u = 6;
    int v = 11;

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

TEST_F(GraphAttrMaskedTest, GetNeighbors_empty)
{
    GraphAttr to;

    ASSERT_EQ(size_t(0), to.VerticesIds().size()) << "Expecting empty graph, without vertices";

    EXPECT_THROW(to.getNeighbors(0), ExceptionKeyError);
    EXPECT_THROW(to.getNeighbors(22), ExceptionKeyError);
    EXPECT_THROW(to.getNeighbors(6), ExceptionKeyError);
    EXPECT_THROW(to.getNeighbors(9), ExceptionKeyError);
    EXPECT_THROW(to.getNeighbors(-1), ExceptionKeyError);
}

TEST_F(GraphAttrMaskedTest, GetNeighbors_invalid)
{
    ASSERT_GT(to_.num_vertices(), static_cast<size_t>(0)) << "this test requires a non-empty graph";

    EXPECT_THROW(to_.getNeighbors(-1), ExceptionKeyError);
    EXPECT_THROW(to_.getNeighbors(0), ExceptionKeyError);
    EXPECT_THROW(to_.getNeighbors(5), ExceptionKeyError);
    EXPECT_THROW(to_.getNeighbors(12), ExceptionKeyError);
    EXPECT_THROW(to_.getNeighbors(9), ExceptionKeyError) << "this vertex was masked.";
}

TEST_F(GraphAttrMaskedTest, GetNeighbors_none)
{
    const int ROWS=3;
    const int COLS=3;
    int data[ROWS*COLS] = {1, 1, 2,
                           3, 1, 6,
                           1, 1, 11};
    map_ = Mat1i(ROWS, COLS, data).clone();

    Mat1b exclude;
    // mask everything around a vertex to get one without neighbors
    cv::bitwise_or(map_ == 1, map_ == 9, exclude);
    cv::bitwise_not(exclude, mask_);

    GraphAttr to = GraphAttr(map_, mask_);

    ASSERT_GT(to.num_vertices(), size_t(1));

    EXPECT_SIZE(0, to.getNeighbors(3));
    EXPECT_GT(to.getNeighbors(6).size(), size_t(0));
}

TEST_F(GraphAttrMaskedTest, GetNeighbors_all_neighbors)
{
    /*
    int data[ROWS*COLS] = {1, 7, 2,
                           3, 6, 6,
                           9, 9, 11};

        cv::bitwise_or(map_ < 2, map_ == 9, exclude);
        { x , 7, 2,
          3 , 6, 6,
          x , x, 11};
    */

    ASSERT_GT(to_.num_vertices(), size_t(1));

    VecI tmp = to_.getNeighbors(6);
    std::set<int> neigh_ids(tmp.begin(), tmp.end());
    EXPECT_SIZE(4, neigh_ids);

    std::set<int> expected_neighs;
    expected_neighs.insert(7);
    expected_neighs.insert(2);
    expected_neighs.insert(3);
    expected_neighs.insert(11);

    EXPECT_EQ(expected_neighs, neigh_ids);
}

TEST_F(GraphAttrMaskedTest, GetNeighbors_who)
{
    /*
    int data[ROWS*COLS] = {1, 7, 2,
                           3, 6, 6,
                           9, 9, 11};

        cv::bitwise_or(map_ < 2, map_ == 9, exclude);
        { x , 7, 2,
          3 , 6, 6,
          x , x, 11};
    */
    ASSERT_GT(to_.num_vertices(), size_t(1));

    {
        VecI tmp = to_.getNeighbors(2);
        std::set<int> neigh_ids(tmp.begin(), tmp.end());
        EXPECT_SIZE(2, neigh_ids);

        std::set<int> expected_neighs;
        expected_neighs.insert(6);
        expected_neighs.insert(7);

        EXPECT_EQ(expected_neighs, neigh_ids);
    }
    {
        VecI tmp = to_.getNeighbors(11);
        std::set<int> neigh_ids(tmp.begin(), tmp.end());
        EXPECT_SIZE(1, neigh_ids);

        std::set<int> expected_neighs;
        expected_neighs.insert(6);

        EXPECT_EQ(expected_neighs, neigh_ids);
    }
    {
        VecI tmp = to_.getNeighbors(3);
        std::set<int> neigh_ids(tmp.begin(), tmp.end());
        EXPECT_SIZE(1, neigh_ids);

        std::set<int> expected_neighs;
        expected_neighs.insert(6);

        EXPECT_EQ(expected_neighs, neigh_ids);
    }
}

TEST_F(GraphAttrMaskedTest, GetNeighbors_adjacency)
{
    /*
    int data[ROWS*COLS] = {1, 7, 2,
                           3, 6, 6,
                           9, 9, 11};

        cv::bitwise_or(map_ < 2, map_ == 9, exclude);
        { x , 7, 2,
          3 , 6, 6,
          x , x, 11};
    */
    ASSERT_GT(to_.num_vertices(), size_t(1));

    Mat1f adj;
    to_.AdjacencyMat(adj);

    VecI vtx_ids = to_.VerticesIds();

    for(size_t i=0; i<vtx_ids.size(); i++) {

        VecI neigh_ids = to_.getNeighbors(vtx_ids[i]);

        EXPECT_SIZE(cv::sum(adj.row(i))[0], neigh_ids);

        for(size_t j=0; j<neigh_ids.size(); j++) {

            EXPECT_FLOAT_EQ(1.f, adj(i, to_.VertexIndex(neigh_ids[j])));
        }
    }
}

TEST_F(GraphAttrMaskedTest, GetNeighbors_mulitple_contacts)
{
    const int ROWS=2;
    const int COLS=3;
    int data[ROWS*COLS] = {3, 1, 2,
                           3, 1, 6};
    map_ = Mat1i(ROWS, COLS, data).clone();

    GraphAttr to = GraphAttr(map_, Mat1b());

    EXPECT_SIZE(1, to.getNeighbors(3));
    EXPECT_SIZE(3, to.getNeighbors(1));
    EXPECT_SIZE(2, to.getNeighbors(2));
}

TEST_F(GraphAttrMaskedTest, Remove_vertex_empty)
{
    GraphAttr to;

    ASSERT_EQ(size_t(0), to.VerticesIds().size()) << "Expecting empty graph, without vertices";

    EXPECT_THROW(to.removeVertex(0), ExceptionKeyError);
    EXPECT_THROW(to.removeVertex(2), ExceptionKeyError);
    EXPECT_THROW(to.removeVertex(6), ExceptionKeyError);
    EXPECT_THROW(to.removeVertex(9), ExceptionKeyError);
    EXPECT_THROW(to.removeVertex(-1), ExceptionKeyError);
}

TEST_F(GraphAttrMaskedTest, Remove_vertex_invalid)
{
    ASSERT_GT(to_.num_vertices(), static_cast<size_t>(0)) << "this test requires a non-empty graph";

    EXPECT_THROW(to_.removeVertex(-1),  ExceptionKeyError);
    EXPECT_THROW(to_.removeVertex(0),   ExceptionKeyError);
    EXPECT_THROW(to_.removeVertex(5),   ExceptionKeyError);
    EXPECT_THROW(to_.removeVertex(12),  ExceptionKeyError);
    EXPECT_THROW(to_.removeVertex(9),   ExceptionKeyError) << "this vertex was masked.";
}

/**
 * @brief Verify which vertex got removed after merge
 */
TEST_F(GraphAttrMaskedTest, Remove_vertex)
{
    ASSERT_GT(static_cast<int>(to_.num_vertices()), 1) << "this test requires a graph with at least 1 vertex.";

    while(static_cast<int>(to_.num_vertices()) > 1) {

        VecI vtx_ids = to_.VerticesIds();

        int u = vtx_ids[1];
        int v = vtx_ids[0];

        to_.removeVertex(u);

        EXPECT_THROW(to_.VertexIndex(u), ExceptionKeyError);
        EXPECT_THROW(to_.removeVertex(u), ExceptionKeyError);
        EXPECT_EQ(0, to_.VertexIndex(v));
    }
}

TEST_F(GraphAttrMaskedTest, Remove_vertex_all_vertices)
{
    ASSERT_GT(static_cast<int>(to_.num_vertices()), 1) << "this test requires a graph with at least 1 vertex.";

    while(static_cast<int>(to_.num_vertices()) > 1) {

        VecI vtx_ids = to_.VerticesIds();

        int v = vtx_ids[0];

        to_.removeVertex(v);

        EXPECT_THROW(to_.VertexIndex(v), ExceptionKeyError);
        EXPECT_THROW(to_.removeVertex(v), ExceptionKeyError);
    }
}

TEST_F(GraphAttrMaskedTest, Remove_vertex_check_vtx_ids)
{
    ASSERT_GT(static_cast<int>(to_.num_vertices()), 1) << "this test requires a graph with at least 1 vertex.";

    while(static_cast<int>(to_.num_vertices()) > 0) {

        VecI vtx_ids = to_.VerticesIds();
        int v = vtx_ids[0];

        to_.removeVertex(v);

        vtx_ids = to_.VerticesIds();
        VecI::const_iterator itr = std::find(vtx_ids.begin(), vtx_ids.end(), v);
        EXPECT_EQ(vtx_ids.end(), itr) << "Vertex still in list of vertex ids after removal.";
    }
}

cv::Mat1f mask_vertex(const cv::Mat1f& img, const cv::Mat1b &mask)
{
    Mat1b mask_inverted;
    cv::bitwise_not(mask, mask_inverted);
    return img.clone().setTo(0, mask_inverted);
}

TEST_F(GraphAttrMaskedTest, Remove_vertex_check_map)
{
    ASSERT_GT(static_cast<int>(to_.num_vertices()), 1) << "this test requires a graph with at least 1 vertex.";

    Mat1f map;

    while(static_cast<int>(to_.num_vertices()) > 0) {

        VecI vtx_ids = to_.VerticesIds();
        int v = vtx_ids[0];

        // replace with getter to Graph's underlying map image
        VecMat1f masked_maps = to_.applyVerticesToMap(mask_vertex);
        map = Mat1f::zeros(map_.size());
        for(size_t i=0; i<masked_maps.size(); i++) {

            map += masked_maps[i];
        }
        EXPECT_GT(countNonZero(map==v), 0);

        to_.removeVertex(v);

        masked_maps = to_.applyVerticesToMap(mask_vertex);
        Mat1f map = Mat1f::zeros(map_.size());
        for(size_t i=0; i<masked_maps.size(); i++) {

            map += masked_maps[i];
        }
        EXPECT_EQ(0, countNonZero(map==v)) << "still finding pixel with vertex color after removal.";

    }
}

TEST_F(GraphAttrMaskedTest, GetMapImg)
{
    EXPECT_MAT_EQ(map_, to_.MapImg());
}

TEST_F(GraphAttrMaskedTest, GetMapImg_vertex_removed)
{
    ASSERT_GT(static_cast<int>(to_.num_vertices()), 2) << "this test requires a graph with at least 2 vertices.";

    VecI vtx_ids = to_.VerticesIds();
    int u = vtx_ids[0];
    int v = vtx_ids[1];

    EXPECT_GT(countNonZero(to_.MapImg()==u), 0);
    EXPECT_GT(countNonZero(to_.MapImg()==v), 0);

    to_.removeVertex(v);

    EXPECT_GT(countNonZero(to_.MapImg()==u), 0);
    EXPECT_EQ(0, countNonZero(to_.MapImg()==v));
}

TEST_F(GraphAttrMaskedTest, GetMapImg_vertex_removed_mask)
{
    ASSERT_GT(static_cast<int>(to_.num_vertices()), 2) << "this test requires a graph with at least 2 vertices.";

    VecI vtx_ids = to_.VerticesIds();
    int u = vtx_ids[0];
    int v = vtx_ids[1];

    Mat1b mask_u = to_.MapImg()==u;
    Mat1b mask_v = to_.MapImg()==v;

    ASSERT_FALSE(Equal(mask_u, mask_v));

    to_.removeVertex(v);

    EXPECT_MAT_EQ(mask_u, to_.MapImg()==u);
    EXPECT_EQ(0, countNonZero(to_.MapImg()==v));
    EXPECT_FALSE(Equal(to_.MapImg()==v, mask_v));
}

} // annonymous namespace for test cases and fixtures

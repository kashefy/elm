#include "elm/core/graph/base_GraphVertexOp.h"

#include <boost/thread.hpp>

#include "gtest/gtest.h"

#include <opencv2/core/core.hpp>

#include "elm/core/exception.h"
#include "elm/core/graph/graphattr.h"
#include "elm/ts/mat_assertions.h"

using namespace cv;
using namespace elm;

namespace {

/** Derive from Graph operator interface to test
  */
class DummyOp : public base_GraphVertexOp
{
public:
    DummyOp()
        : base_GraphVertexOp(),
          call_count_(0)
    {
    }

    Mat1f mutableOp(const Mat1i &img, const Mat &mask)
    {
        ++call_count_;
        return img.clone().setTo(0, mask);
    }

    int CallCount() const
    {
        return call_count_;
    }

protected:
    int call_count_;    ///< increments with every operator call
};

class GraphVertexOpTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        to_ = DummyOp();
    }

    DummyOp to_;    ///< test object
};

TEST_F(GraphVertexOpTest, op)
{
    Mat1i img(2, 2);
    for(size_t i=0; i<img.total(); i++) {

        img(i) = i;
    }

    Mat1f result;

    for(size_t i=0; i<img.total(); i++) {

        result = to_.mutableOp(img, img == static_cast<float>(i));

        EXPECT_FLOAT_EQ(0.f, result(i));

        Mat1f expected = img.clone();
        expected(i) = 0.f;

        EXPECT_MAT_EQ(expected, result);
    }
}

TEST_F(GraphVertexOpTest, op_input_empty)
{
    EXPECT_TRUE(to_.mutableOp(Mat1f(), Mat()).empty());
}

TEST_F(GraphVertexOpTest, op_dims)
{
    for(int r=1; r<11; r++) {

        for(int c=1; c<11; c++) {

            Mat1i img(r, c, 123);
            Mat1f result = to_.mutableOp(img, img > 0);
            EXPECT_MAT_DIMS_EQ(result, cv::Size2i(c, r));
        }
    }
}

TEST_F(GraphVertexOpTest, op_mask)
{
    Mat1i img(2, 2);
    for(size_t i=0; i<img.total(); i++) {

        img(i) = i;
    }

    Mat1f result;

    for(size_t i=0; i<img.total(); i++) {

        result = to_.mutableOp(img, img == static_cast<float>(i));

        EXPECT_FLOAT_EQ(0.f, result(i));

        Mat1f expected = img.clone();
        expected(i) = 0.f;

        EXPECT_MAT_EQ(expected, result);
    }
}

TEST_F(GraphVertexOpTest, op_call_count)
{
    Mat1i img(2, 2, 1);

    Mat1f result;

    for(size_t i=0; i<img.total(); i++) {

        result = to_.mutableOp(img, img == static_cast<int>(i));
        EXPECT_EQ(static_cast<int>(i+1), to_.CallCount());
    }
}

TEST_F(GraphVertexOpTest, op_applied_to_graph_vertex)
{
    const int ROWS=3;
    const int COLS=3;
    int data[ROWS*COLS] = {1, 7, 2,
                           3, 6, 6,
                           9, 9, 11};
    Mat1i map = Mat1i(ROWS, COLS, data).clone();

    Mat1b exclude, mask;
    cv::bitwise_or(map < 2, map == 9, exclude);
    cv::bitwise_not(exclude, mask);

    GraphAttr graph(map, mask);

    ASSERT_GT(graph.num_vertices(), static_cast<size_t>(0)) << "this test requires a non-empty graph";

    Mat1f result_graph = graph.applyVertexOpToMap(6, to_);

    Mat1f result_op = to_.mutableOp(map, map == 6);

    EXPECT_MAT_EQ(result_op, result_graph);
}

TEST_F(GraphVertexOpTest, op_applied_to_graph_vertex_invalid_vtx)
{
    const int ROWS=3;
    const int COLS=3;
    int data[ROWS*COLS] = {1, 7, 2,
                           3, 6, 6,
                           9, 9, 11};
    Mat1i map = Mat1i(ROWS, COLS, data).clone();

    Mat1b exclude, mask;
    cv::bitwise_or(map < 2, map == 9, exclude);
    cv::bitwise_not(exclude, mask);

    GraphAttr graph(map, mask);

    ASSERT_GT(graph.num_vertices(), static_cast<size_t>(0)) << "this test requires a non-empty graph";

    EXPECT_THROW(graph.applyVertexOpToMap(-1, to_), ExceptionKeyError);
    EXPECT_THROW(graph.applyVertexOpToMap(0, to_), ExceptionKeyError);
    EXPECT_THROW(graph.applyVertexOpToMap(5, to_), ExceptionKeyError);
    EXPECT_THROW(graph.applyVertexOpToMap(12, to_), ExceptionKeyError);
}

TEST_F(GraphVertexOpTest, op_applied_to_graph_vertex_call_count)
{
    const int ROWS=3;
    const int COLS=3;
    int data[ROWS*COLS] = {1, 7, 2,
                           3, 6, 6,
                           9, 9, 11};
    Mat1i map = Mat1i(ROWS, COLS, data).clone();

    Mat1b exclude, mask;
    cv::bitwise_or(map < 2, map == 9, exclude);
    cv::bitwise_not(exclude, mask);

    GraphAttr graph(map, mask);

    ASSERT_GT(graph.num_vertices(), static_cast<size_t>(0)) << "this test requires a non-empty graph";

    EXPECT_EQ(0, to_.CallCount());

    graph.applyVertexOpToMap(6, to_);

    EXPECT_EQ(1, to_.CallCount());
}

} // annonymous namespace for tests

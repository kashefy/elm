#include "elm/core/graph/base_GraphVertexOp.h"

#include "gtest/gtest.h"

#include <opencv2/core/core.hpp>

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

    cv::Mat1f operator ()(const cv::Mat1f& img, const cv::Mat1b &mask)
    {
        call_count_++;
        return img.clone().setTo(0.f, mask);
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
    Mat1f img(2, 2);
    for(size_t i=0; i<img.total(); i++) {

        img(i) = static_cast<float>(i);
    }

    Mat1f result;

    for(size_t i=0; i<img.total(); i++) {

        result = to_(img, img == static_cast<float>(i));

        EXPECT_FLOAT_EQ(0.f, result(i));

        Mat1f expected = img.clone();
        expected(i) = 0.f;

        EXPECT_MAT_EQ(expected, result);
    }
}

TEST_F(GraphVertexOpTest, op_input_empty)
{
    EXPECT_TRUE(to_(Mat1f(), Mat1b()).empty());
}

TEST_F(GraphVertexOpTest, op_dims)
{
    for(int r=1; r<11; r++) {

        for(int c=1; c<11; c++) {

            Mat1f img(r, c, 123.f);
            Mat1f result = to_(img, img > 0);
            EXPECT_MAT_DIMS_EQ(result, cv::Size2i(c, r));
        }
    }
}

TEST_F(GraphVertexOpTest, op_mask)
{
    Mat1f img(2, 2);
    for(size_t i=0; i<img.total(); i++) {

        img(i) = static_cast<float>(i);
    }

    Mat1f result;

    for(size_t i=0; i<img.total(); i++) {

        result = to_(img, img == static_cast<float>(i));

        EXPECT_FLOAT_EQ(0.f, result(i));

        Mat1f expected = img.clone();
        expected(i) = 0.f;

        EXPECT_MAT_EQ(expected, result);
    }
}

TEST_F(GraphVertexOpTest, op_call_count)
{
    Mat1f img(2, 2, 1.f);

    Mat1f result;

    for(size_t i=0; i<img.total(); i++) {

        result = to_(img, img == static_cast<float>(i));
        EXPECT_EQ(static_cast<int>(i+1), to_.CallCount());
    }
}

} // annonymous namespace for tests

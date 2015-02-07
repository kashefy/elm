/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/core/graph/graph.h"

#include "gtest/gtest.h"

#ifdef __WITH_PCL

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/Vertices.h>

#endif // __WITH_PCL

#include "elm/core/exception.h"

using namespace elm;

namespace {

class GraphConstructTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
    }

    virtual void TearDown()
    {
    }
};

TEST_F(GraphConstructTest, Consturct)
{
    EXPECT_NO_THROW(Graph());
    EXPECT_NO_THROW(Graph(0));
    EXPECT_NO_THROW(Graph(1));
}

TEST_F(GraphConstructTest, Destruct)
{
    Graph *to = new Graph;
    EXPECT_NO_THROW(delete to);
}

#ifdef __WITH_PCL

TEST_F(GraphConstructTest, FromTriangulatedPointCloud)
{
    using namespace pcl;
    CloudXYZPtr cld;
    Triangles tri;

    Graph g(cld, tri);
}

} // annonymous namespace for unit tests

#endif // __WITH_PCL

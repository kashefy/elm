#include "elm/core/graph/adjacency.h"

#ifdef __WITH_PCL

#include <pcl/point_cloud.h>
#include <pcl/Vertices.h>

#include "elm/core/exception.h"
#include "elm/ts/mat_assertions.h"

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
    EXPECT_THROW(TriangulatedCloudToAdjacencyMat(cld, tri), ExceptionNotImpl);
}


} // annonymous namespace for tests

#endif // __WITH_PCL

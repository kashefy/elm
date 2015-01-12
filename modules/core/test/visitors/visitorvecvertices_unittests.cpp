/** @file test out VecVertices visitor
 */
#ifdef __WITH_PCL // PCL support required for these tests

#include "core/visitors/visitorvertices.h"

#include "core/exception.h"
#include "ts/ts.h"

using namespace std;
using namespace cv;
using namespace sem;

namespace {

/**
 * @brief test class around VisitorVecVertices
 */
class VisitorVecVerticesTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        to_.Reset();
    }

    VisitorVecVertices to_;    ///< test object
};

TEST_F(VisitorVecVerticesTest, Empty)
{
}

} // annonymous namespace for VecVertices visitors' test fixtures

#else // __WITH_PCL
    #warning "Skipping building VecVertices visitor unit tests due to no pcl support."
#endif // __WITH_PCL

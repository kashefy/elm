#include "elm/layers/gradassignment.h"
#include "elm/layers/triangulation.h"

#ifdef __WITH_PCL

#include <boost/filesystem.hpp>

#include "elm/layers/layerfactory.h"
#include "elm/ts/ts.h"

using namespace std;
namespace bfs=boost::filesystem;
using namespace cv;
using namespace elm;

namespace {

const bfs::path TEST_DIR("testdata");
const bfs::path TEST_PATH_PCD = TEST_DIR/"bun0.pcd";

/**
 * @brief Test graph matching feature(s)
 */
class GraphMatchingTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {

    }

    virtual void TearDown()
    {

    }

    // members
    std::vector<std::shared_ptr<base_Layer > > layers_;
};




} // annonymous namespace for test cases and test fixtures

#endif // __WITH_PCL

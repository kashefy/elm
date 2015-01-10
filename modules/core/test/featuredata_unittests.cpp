#include "core/featuredata.h"

#include "core/core.h"
#include "ts/ts.h"

using namespace std;
using namespace cv;
using namespace sem;

namespace {

class FeatureDataTest : public ::testing::Test
{

};

TEST_F(FeatureDataTest, Foo)
{
    Mat_<float> m(1, 3, 2);
    FeatureData x(m);
    PointCloudXYZ::Ptr cld = x.get<PointCloudXYZ::Ptr>();


    cld = Mat2PointCloud(m);
    FeatureData y(cld);

    Mat_<float> m2 = y.get<Mat_<float> >();
}

} // annonymous namespace for test fixtures

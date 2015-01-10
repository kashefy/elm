#include "core/featuredata.h"

#include "core/exception.h"
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
    Mat_f m(1, 3, 2);
    FeatureData x(m);
    CloudXYZ::Ptr cld = x.get<CloudXYZ::Ptr>();


    cld = Mat2PointCloud(m);
    FeatureData y(cld);

    Mat_f m2 = y.get<Mat_f >();
}

} // annonymous namespace for test fixtures

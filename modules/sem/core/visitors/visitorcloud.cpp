#include "sem/core/visitors/visitorcloud.h"

#ifdef __WITH_PCL // definitions below require PCL support

#include "sem/core/exception.h"
#include "sem/core/pcl/cloud_.h"
#include "sem/core/pcl/vertices.h"

using namespace pcl;
using namespace sem;

void VisitorCloud::Reset()
{
    c_.reset();
}

CloudXYZPtr VisitorCloud::operator()(CloudXYZPtr &c)
{
    if(!bool(c_)) {

        c_ = c;
    }
    return c_;
}

CloudXYZPtr VisitorCloud::operator()(const VecVertices &vv)
{
    if(!bool(c_)) {

        c_ = Mat2PointCloud_<PointXYZ>(VecVertices2Mat(vv, false));
    }
    return c_;
}

CloudXYZPtr VisitorCloud::operator()(float f)
{
    Reset();
    SEM_THROW_TYPE_ERROR("Cannot create point cloud from float");
}

CloudXYZPtr VisitorCloud::operator()(int n)
{
    Reset();
    SEM_THROW_TYPE_ERROR("Cannot create point cloud from int");
}

CloudXYZPtr VisitorCloud::operator()(uchar c)
{
    Reset();
    SEM_THROW_TYPE_ERROR("Cannot create point cloud from uchar");
}

CloudXYZPtr VisitorCloud::operator()(const Mat_f &m)
{
    if(!bool(c_)) {

        c_ = Mat2PointCloud_<PointXYZ>(m);
    }
    return c_;
}

#endif // __WITH_PCL

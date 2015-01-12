#include "core/visitors/visitorcloud.h"

#ifdef __WITH_PCL // definitions below require PCL support

#include "core/exception.h"

using namespace sem;

void VisitorCloud::Reset()
{
    c_.reset();
}

CloudXYZ::Ptr VisitorCloud::operator()(CloudXYZ::Ptr &c)
{
    if(!bool(c_)) {

        c_ = c;
    }
    return c_;
}

CloudXYZ::Ptr VisitorCloud::operator()(const VecVertices &vv)
{
    if(!bool(c_)) {

        c_ = Mat2PointCloud(VecVertices2Mat(vv, false));
    }
    return c_;
}

CloudXYZ::Ptr VisitorCloud::operator()(float f)
{
    Reset();
    SEM_THROW_TYPE_ERROR("Cannot create point cloud from float");
}

CloudXYZ::Ptr VisitorCloud::operator()(int n)
{
    Reset();
    SEM_THROW_TYPE_ERROR("Cannot create point cloud from int");
}

CloudXYZ::Ptr VisitorCloud::operator()(uchar c)
{
    Reset();
    SEM_THROW_TYPE_ERROR("Cannot create point cloud from uchar");
}

CloudXYZ::Ptr VisitorCloud::operator()(const Mat_f &m)
{
    if(!bool(c_)) {

        c_ = Mat2PointCloud(m);
    }
    return c_;
}

#endif // __WITH_PCL

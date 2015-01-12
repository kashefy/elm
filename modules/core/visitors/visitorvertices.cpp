#include "core/visitors/visitorvertices.h"

#ifdef __WITH_PCL // definitions below require PCL support

using namespace sem;

// imeplemnt VecVertices visitor methods
void VisitorVecVertices::Reset()
{
    vv_.clear();
}

VecVertices VisitorVecVertices::operator()(const VecVertices &vv)
{
    if(vv_.empty()) {

        vv_ = vv;
    }
    return vv_;
}

VecVertices VisitorVecVertices::operator()(CloudXYZ::Ptr &c)
{
    if(vv_.empty()) {

        vv_ = Mat2VecVertices(PointCloud2Mat(c)); // or keep cache?
    }
    return vv_;
}

VecVertices VisitorVecVertices::operator()(float f)
{
    FromScalar(f);
    return vv_;
}

VecVertices VisitorVecVertices::operator()(int n)
{
    FromScalar(n);
    return vv_;
}

VecVertices VisitorVecVertices::operator()(uchar c)
{
    FromScalar(c);
    return vv_;
}

VecVertices VisitorVecVertices::operator()(const Mat_f &m)
{
    if(vv_.empty()) {

        vv_ = Mat2VecVertices(m);
    }
    return vv_;
}

#endif // __WITH_PCL

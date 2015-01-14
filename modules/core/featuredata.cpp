#include "core/featuredata.h"

using namespace std;
using namespace cv;
using namespace sem;

FeatureData::FeatureData()
{
    Init();
}

// explicit specialization for template method get()

template<>
Mat_<float> FeatureData::get()
{
    return var_.apply_visitor(visitor_mat_);
}

#ifdef __WITH_PCL // PCL support required
template<>
CloudXYZPtr FeatureData::get()
{
    return var_.apply_visitor(visitor_cloud_);
}

template<>
VecVertices FeatureData::get()
{
    return var_.apply_visitor(visitor_vv_);
}
#endif // __WITH_PCL

#define IMPLEMENT_STATELESS_GET(_TYP) template<> _TYP FeatureData::get() {  \
    return boost::apply_visitor(VisitorPOD_<_TYP>(), var_);                 \
                                                                         }

IMPLEMENT_STATELESS_GET(float)
IMPLEMENT_STATELESS_GET(int)
IMPLEMENT_STATELESS_GET(uchar)

void FeatureData::Init()
{
    Reset();
}

void FeatureData::Reset()
{
    visitor_cloud_.Reset();
    visitor_mat_.Reset();
}

std::ostream& operator<<(std::ostream& os, FeatureData& obj)
{
    os << obj.get<Mat_<float> >();
    return os;
}

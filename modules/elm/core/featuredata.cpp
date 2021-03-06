/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/core/featuredata.h"

#include <iostream>

#include "elm/core/visitors/visitormat1f.h"
#include "elm/core/visitors/visitorvecmat1f.h"
#include "elm/core/visitors/visitorsparsemat1f.h"
#include "elm/core/visitors/visitorpod_.h"

using namespace std;
using namespace cv;

template class cv::Mat_<float>;

namespace elm {

FeatureData::FeatureData()
{
    Init();
}

// explicit specializations for template method get()

template<>
Mat_<float> FeatureData::get()
{
    //return var_.apply_visitor(visitor_mat_);
    return boost::apply_visitor(VisitorMat1f(), var_);
}

template<>
SparseMat1f FeatureData::get()
{
    //return var_.apply_visitor(visitor_sparse_mat_);
    return boost::apply_visitor(VisitorSparseMat1f(), var_);
}

template<>
VecMat1f FeatureData::get()
{
    //return var_.apply_visitor(visitor_vm_);
    return boost::apply_visitor(VisitorVecMat1f(), var_);
}

#ifdef __WITH_PCL // PCL support required
template<>
CloudXYZPtr FeatureData::get()
{
    return var_.apply_visitor(visitor_cloud_xyz_);
}

template<>
CloudNrmlPtr FeatureData::get()
{
    return var_.apply_visitor(visitor_cloud_nrml_);
}

template<>
CloudPtNrmlPtr FeatureData::get()
{
    return var_.apply_visitor(visitor_cloud_ptnrml_);
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
    visitor_cloud_nrml_.Reset();
    visitor_cloud_ptnrml_.Reset();
    visitor_cloud_xyz_.Reset();
    visitor_vv_.Reset();
    //visitor_mat_.Reset();
    //visitor_sparse_mat_.Reset();
    //visitor_vm_.Reset();
}

std::ostream& operator<<(std::ostream& os, FeatureData& obj)
{
    os << obj.get<Mat_<float> >();
    return os;
}

} // namespace elm


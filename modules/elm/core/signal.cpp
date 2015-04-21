/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/core/signal.h"

#include "elm/core/exception.h"
#include "elm/core/signal_impl.h"
#include "elm/core/stl/stl_inl.h"

using namespace std;
using namespace cv;
using namespace elm;

Signal::~Signal() {

    impl_.reset();
}

Signal::Signal() {

    impl_.reset(new Signal_Impl());
}

VecMat Signal::operator [](const string &name) const
{
    return impl_->operator [](name);
}

Mat1f Signal::MostRecentMat1f(const string &name) const
{
    return impl_->MostRecentMat1f(name);
}

void Signal::Clear() {

    impl_->Clear();
}

void Signal::Append(const std::string &name, const FeatureData &feat) {

    impl_->Append(name, feat);
}

void Signal::Append(const std::string &name, const cv::Mat1f &feat) {

    impl_->Append(name, feat);
}

void Signal::Append(const std::string &name, const VecMat1f &feat) {

    impl_->Append(name, feat);
}

void Signal::Append(const std::string &name, const SparseMat1f &feat) {

    impl_->Append(name, feat);
}

void Signal::Append(const std::string &name, const CloudXYZPtr &feat) {

    impl_->Append(name, feat);
}

bool Signal::Exists(const std::string &name) const {

    return impl_->Exists(name);
}

VecS Signal::FeatureNames() const {

    return impl_->FeatureNames();
}

FeatureData Signal::MostRecent(const std::string& name) const {

    return impl_->MostRecent(name);
}


/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/core/signal.h"

#include "elm/core/exception.h"
#include "elm/core/stl/stl.h"

using namespace std;
using namespace cv;
using namespace elm;

Signal::~Signal()
{
}

Signal::Signal() :
    Signal_<FeatureData >()
{
}

VecMat Signal::operator [](const string &name) const
{
    MapSVecFD::const_iterator itr = signals_.find(name);
    if(itr != signals_.end()) { // found

        VecFeatData vf = itr->second;
        VecMat v;
        for(size_t i=0; i < vf.size(); i++) {

            Mat1f m = vf[i].get<Mat1f>();
            v.push_back(m);
        }
        return v;
    }
    else {
        stringstream s;
        s << "Feature \'" << name << "\' does not exist.";
        ELM_THROW_KEY_ERROR(s.str());
    }
}

Mat1f Signal::MostRecentMat1f(const string &name) const
{
    return static_cast<FeatureData>(MostRecent(name)).get<Mat1f>();
}


#include "core/signal.h"

#include "core/exception.h"
#include "core/stl.h"

using namespace std;
using namespace cv;

Signal::~Signal()
{
}

Signal::Signal()
{
}

void Signal::Clear()
{
    signals_.clear();
}

void Signal::Append(const string &name, const MatExpr &feature_data)
{
    Append(name, Mat(feature_data));
}

void Signal::Append(const string &name, const FeatureData &feature_data)
{
    map<string, VecFeatData >::iterator itr = signals_.find(name);
    if(itr != signals_.end()) { // found

        itr->second.push_back(feature_data);
    }
    else { // not found, new insertion

        signals_[name] = VecFeatData(1, feature_data);
    }
}

bool Signal::Exists(const string &name) const
{
    VecFeatData tmp;
    return sem::Find(signals_, name, tmp);
}

VecS Signal::FeatureNames() const
{
    VecS feature_names;
    for(map<string, VecFeatData >::const_iterator itr=signals_.begin();
        itr != signals_.end();
        ++itr) {

        feature_names.push_back(itr->first);
    }
    return feature_names;
}

VecFeatData Signal::GetFeatureData(const string &name) const
{
    MapSVecFD::const_iterator itr = signals_.find(name);
    if(itr != signals_.end()) { // found

        return itr->second;
    }
    else {
        stringstream s;
        s << "Feature \'" << name << "\' does not exist.";
        SEM_THROW_KEY_ERROR(s.str());
    }
}

VecMat Signal::operator [](const string &name) const
{
    MapSVecFD::const_iterator itr = signals_.find(name);
    if(itr != signals_.end()) { // found

        VecFeatData vf = itr->second;
        VecMat v;
        for(size_t i=0; i < vf.size(); i++) {

            Mat_f m = vf[i].get<Mat_f>();
            v.push_back(m);
        }
        return v;
    }
    else {
        stringstream s;
        s << "Feature \'" << name << "\' does not exist.";
        SEM_THROW_KEY_ERROR(s.str());
    }
}

Mat Signal::MostRecent(const string &name) const
{
    MapSVecFD::const_iterator itr = signals_.find(name);
    if(itr != signals_.end()) { // found

        size_t len = itr->second.size();
        if(len > 0) {

            return static_cast<FeatureData>(itr->second[len-1]).get<Mat_f>();
        }
        else {
            stringstream s;
            s << "Feature \'" << name << "\' is empty.";
            SEM_THROW_BAD_DIMS(s.str());
        }
    }
    else {
        stringstream s;
        s << "Feature \'" << name << "\' does not exist.";
        SEM_THROW_KEY_ERROR(s.str());
    }
}


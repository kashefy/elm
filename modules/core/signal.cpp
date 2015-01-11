#include "core/signal.h"

#include "core/exception.h"
#include "core/stl.h"

using namespace std;
using namespace cv;

Signal::~Signal()
{
}

Signal::Signal() :
    Signal_<FeatureData >()
{
}

//void Signal::Append(const string &name, const MatExpr &feature_data)
//{
//    Append(name, FeatureData(Mat(feature_data)));
//}

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

Mat Signal::MostRecentMat(const string &name) const
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


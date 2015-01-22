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

            Mat_f m = vf[i].get<Mat_f>();
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

Mat1f Signal::MostRecentMat(const string &name) const
{
    return static_cast<FeatureData>(MostRecent(name)).get<Mat_f>();
}


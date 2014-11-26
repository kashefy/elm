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

void Signal::Append(const string &name, const Mat &feature_data)
{
    map<string, VecMat >::iterator itr = signals_.find(name);
    if(itr != signals_.end()) { // found

        itr->second.push_back(feature_data);
    }
    else { // not found, new insertion

        signals_[name] = VecMat(1, feature_data);
    }
}

bool Signal::Exists(const string &name) const
{
    VecMat tmp;
    return sem::Find(signals_, name, tmp);
}

VecS Signal::FeatureNames() const
{
    VecS feature_names;
    for(map<string, VecMat >::const_iterator itr=signals_.begin();
        itr != signals_.end();
        ++itr) {

        feature_names.push_back(itr->first);
    }
    return feature_names;
}



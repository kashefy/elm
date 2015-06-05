/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/python/signalpy.h"

#include <boost/python/extract.hpp>
#include <boost/python/list.hpp>
#include <boost/python/str.hpp>

#define PY_ARRAY_UNIQUE_SYMBOL COOL_ARRAY_API
#define NO_IMPORT_ARRAY
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <numpy/ndarrayobject.h>

#include <opencv2/core/core.hpp>

#include "elm/python/arginfo.h"
#include "elm/python/stl_inl.h"

using std::string;
namespace bp=boost::python;
using namespace cv;
using namespace elm;

SignalPy::SignalPy()
    : Signal()
{
}

bp::dict SignalPy::toPythonDict() const {

    bp::dict d;
    VecS names = FeatureNames();

    for(VecS::const_iterator itr = names.begin();
        itr != names.end();
        ++itr) {

        string n = *itr;
        VecMat feats = this->operator [](n);
        d[n] = toPythonList(feats);
    }

    return d;
}

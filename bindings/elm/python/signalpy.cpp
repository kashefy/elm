/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/python/signalpy.h"

#include <opencv2/core/core.hpp>

#include "elm/python/arginfo.h"

using std::string;
namespace bp=boost::python;
using namespace cv;
using namespace elm;

SignalPy::SignalPy()
    : Signal()
{
}

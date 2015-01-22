/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, Elm Library Project
// 3-clause BSD License
//
//M*/
#include "elm/core/signal_.h"

#include "elm/core/exception.h"
#include "elm/ts/signal_tp_.h"
#include "elm/ts/ts.h"

using namespace std;
using namespace cv;
using namespace elm;

typedef ::testing::Types<int, cv::Mat1f, float> SignalFeatType;
INSTANTIATE_TYPED_TEST_CASE_P(SignalTPTests, Signal_TP_, SignalFeatType);

template<> std::vector<int> FtV_<int>::values{0, 1, 100};
template<> std::vector<cv::Mat1f > FtV_< cv::Mat1f >::values{Mat1f(1, 1, 0.f), Mat1f(1, 1, 1.f), Mat1f(1, 1, 100.f)};
template<> std::vector<float> FtV_<float>::values{0.1f, 0.2f, 100.3f};


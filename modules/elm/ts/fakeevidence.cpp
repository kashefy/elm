/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/ts/fakeevidence.h"

using namespace cv;

FakeEvidence::FakeEvidence(int nb_features)
    : nb_features_(nb_features)
{}

cv::Mat FakeEvidence::next(int state) {

    cv::Mat1i f = cv::Mat1i::zeros(1, nb_features_);
    for(int i=state % 2; i<nb_features_; i+=2) {

        f(i)++;
    }
    return f;
}

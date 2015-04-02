/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/io/matlabmatfilereader.h"

#include "matio.h"

using namespace elm;

MatlabMATFileReader::MatlabMATFileReader()
{
}

MatlabMATFileReader::~MatlabMATFileReader()
{
    //mat_t matfp_;
    if(matfp_ != NULL) {

        Mat_Close(matfp_);
    }
}

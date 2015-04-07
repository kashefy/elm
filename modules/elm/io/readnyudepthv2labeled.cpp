/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/io/readnyudepthv2labeled.h"

#include <vector>

#include <opencv2/imgproc/imgproc.hpp>

#include <elm/core/exception.h>
#include <elm/io/matio_utils.h>
#include <elm/io/matlabmatfilereader.h>

using namespace std;
using namespace cv;
using namespace elm;

const string ReadNYUDepthV2Labeled::KEY_DEPTHS  = "depths";
const string ReadNYUDepthV2Labeled::KEY_IMAGES  = "images";
const string ReadNYUDepthV2Labeled::KEY_LABELS  = "labels";

ReadNYUDepthV2Labeled::ReadNYUDepthV2Labeled()
    : index_(0),
      nb_items_(-1)
{
    reader_ = new MatlabMATFileReader();
}

ReadNYUDepthV2Labeled::~ReadNYUDepthV2Labeled()
{
    if(reader_ != NULL) {

        delete reader_;
        reader_ = NULL;
    }
}

int ReadNYUDepthV2Labeled::ReadHeader(const std::string &path)
{
    reader_->ReadHeader(path);
    vector<string> var_names = reader_->TopLevelVarNames();

    if(std::find(var_names.begin(), var_names.end(), KEY_DEPTHS) == var_names.end()) {

        ELM_THROW_KEY_ERROR("Could not find depths variable.");
    }

    if(std::find(var_names.begin(), var_names.end(), KEY_IMAGES) == var_names.end()) {

        ELM_THROW_KEY_ERROR("Could not find images variable.");
    }

    if(std::find(var_names.begin(), var_names.end(), KEY_LABELS) == var_names.end()) {

        ELM_THROW_KEY_ERROR("Could not find labels variable.");
    }

    reader_->Seek(KEY_DEPTHS);
    depths_ = reader_->CursorToMat();

    reader_->Seek(KEY_IMAGES);
    images_ = reader_->CursorToMat();

    reader_->Seek(KEY_LABELS);
    labels_ = reader_->CursorToMat();

    return labels_.size[labels_.dims-1];
}

void ReadNYUDepthV2Labeled::Next(Mat &bgr, Mat &depth, Mat &labels)
{
    elm::SliceCopy(depths_, 2, index_, depth);

    elm::SliceCopy(labels_, 2, index_, labels);
    labels.convertTo(labels, CV_32SC1);

    Mat bgr3d;
    elm::SliceCopy(images_, 3, index_, bgr3d);
    elm::Mat3DTo3Ch(bgr3d, bgr);
    bgr.convertTo(bgr, CV_8UC3);
    cv::cvtColor(bgr, bgr, CV_RGB2BGR);

    index_++;
}


bool ReadNYUDepthV2Labeled::IS_EOF() const
{
    return index_ >= nb_items_;
}

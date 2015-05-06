/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/io/readnyudepthv2labeled.h"

#ifdef __WITH_MATIO

#include <vector>

#include <opencv2/imgproc/imgproc.hpp>

#include "elm/core/debug_utils.h"

#include "elm/core/signal.h"
#include "elm/core/exception.h"
#include "elm/core/layeroutputnames.h"
#include "elm/io/matio_utils.h"
#include "elm/io/matlabmatfilereader.h"
#include "elm/ts/layerattr_.h"

using namespace std;
using namespace cv;
using namespace elm;

const string ReadNYUDepthV2Labeled::KEY_DEPTHS  = "depths";
const string ReadNYUDepthV2Labeled::KEY_IMAGE  = "images";
const string ReadNYUDepthV2Labeled::KEY_LABELS  = "labels";

#include <boost/assign/list_of.hpp>
template <>
elm::MapIONames LayerAttr_<ReadNYUDepthV2Labeled>::io_pairs = boost::assign::map_list_of
        ELM_ADD_OUTPUT_PAIR(ReadNYUDepthV2Labeled::KEY_DEPTHS)
        ELM_ADD_OUTPUT_PAIR(ReadNYUDepthV2Labeled::KEY_IMAGE)
        ELM_ADD_OUTPUT_PAIR(ReadNYUDepthV2Labeled::KEY_LABELS);

ReadNYUDepthV2Labeled::ReadNYUDepthV2Labeled()
    : base_Reader()
{
    reader_ = new MatlabMATFileReader();
}

ReadNYUDepthV2Labeled::ReadNYUDepthV2Labeled(const LayerConfig &cfg)
    : base_Reader(cfg)
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

    if(std::find(var_names.begin(), var_names.end(), KEY_IMAGE) == var_names.end()) {

        ELM_THROW_KEY_ERROR("Could not find images variable.");
    }

    if(std::find(var_names.begin(), var_names.end(), KEY_LABELS) == var_names.end()) {

        ELM_THROW_KEY_ERROR("Could not find labels variable.");
    }

    reader_->Seek(KEY_DEPTHS);
    depths_ = reader_->CursorToMat();

//    int i=0;
//    for(int r=0; r<640; r++) {

//        for(int c=0; c<480; c++) {

//            std::cout<<depths_.at<float>(i);

//            if(c<480-1) std::cout<<",";
//            else std::cout<<";"<<std::endl;
//            i++;
//        }
//    }

//    std::cout<<std::endl;

    reader_->Seek(KEY_IMAGE);
    images_ = reader_->CursorToMat();

    reader_->Seek(KEY_LABELS);
    labels_ = reader_->CursorToMat();

    nb_items_ = labels_.size[labels_.dims-1];
    return nb_items_;
}

void ReadNYUDepthV2Labeled::Next(Mat &bgr, Mat &depth, Mat &labels)
{
    int idx = labels_.size[2] - Nb_Items();

    //ELM_COUT_VAR(depths_.dims<<":"<<depths_.size[0]<<","<<depths_.size[1]<<","<<depths_.size[2]);

    depth = Mat(depths_.size[1], depths_.size[0], CV_32FC1);
    for(int r=0, i=idx*(depth.total()), j=0; r<depth.rows; r++) {

        for(int c=0; c<depth.cols; c++, i++, j++) {

            depth.at<float>(j) = depths_.at<float>(i);
        }
    }
    //elm::SliceCopy(depths_, 2, index_, depth);
    cv::transpose(depth, depth);

    labels = Mat(labels_.size[1], labels_.size[0], CV_16UC1);

    for(int r=0, i=idx*(labels.total()), j=0; r<labels.rows; r++) {

        for(int c=0; c<labels.cols; c++, i++, j++) {

            labels.at<uint16_t>(j) = labels_.at<uint16_t>(i);
        }
    }
    labels.convertTo(labels, CV_32SC1);

    cv::transpose(labels, labels);

    Mat bgr3d;
    elm::SliceCopy(images_, 3, idx, bgr3d);
    elm::Mat3DTo3Ch(bgr3d, bgr);
    bgr.convertTo(bgr, CV_8UC3);
    cv::cvtColor(bgr, bgr, CV_RGB2BGR);

    nb_items_--;
}

void ReadNYUDepthV2Labeled::OutputNames(const LayerOutputNames &io) {

    name_out_depths_ = io.Output(KEY_DEPTHS);
    name_out_image_ = io.Output(KEY_IMAGE);
    name_out_labels_ = io.Output(KEY_LABELS);
}

void ReadNYUDepthV2Labeled::Next(Signal &signal) {

    Mat bgr, depth, labels;
    Next(bgr, depth, labels);
    nb_items_++; // counter decrement in previous call

    bgr.convertTo(bgr, CV_32FC3);
    depth.convertTo(depth, CV_32FC1);
    labels.convertTo(labels, CV_32FC1);

    signal.Append(name_out_depths_, static_cast<Mat1f>(depth));
    signal.Append(name_out_image_, static_cast<Mat1f>(bgr));
    signal.Append(name_out_labels_, static_cast<Mat1f>(labels));
}

#endif // __WITH_MATIO

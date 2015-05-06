/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/io/readmnistimages.h"

#include <iostream>
#include <sstream>

#include "elm/core/exception.h"
#include "elm/core/signal.h"
#include "elm/io/binary.h"
#include "elm/ts/layerattr_.h"

using namespace std;
using namespace elm;

#include <boost/assign/list_of.hpp>
template <>
elm::MapIONames LayerAttr_<ReadMNISTImages>::io_pairs = boost::assign::map_list_of
        ELM_ADD_OUTPUT_PAIR(detail::BASE_READERMNISTFILE_KEY_OUTPUT);

ReadMNISTImages::ReadMNISTImages()
    : base_ReaderMNISTFile()
{
}

int ReadMNISTImages::ReadHeader(const string &path)
{
    base_ReaderMNISTFile::ReadHeader(path);

    int32_t tmp_i;
    input_.read(reinterpret_cast<char*>(&tmp_i), sizeof(int32_t));
    if(IS_32_LITTLE_ENDIAN) { SwapEndian<int32_t>(&tmp_i); }
    if(tmp_i < 0) {

        ELM_THROW_FILEIO_ERROR("No. of rows must be >= 0");
    }
    rows_ = static_cast<int>(tmp_i);

    input_.read(reinterpret_cast<char*>(&tmp_i), sizeof(int32_t));
    if(IS_32_LITTLE_ENDIAN) { SwapEndian<int32_t>(&tmp_i); }
    if(tmp_i < 0) {

        ELM_THROW_FILEIO_ERROR("No. of columns must be >= 0");
    }
    cols_ = static_cast<int>(tmp_i);

    return nb_items_;
}

int ReadMNISTImages::MagicNumber() const {

    return MAGIC_NUMBER;
}

cv::Mat ReadMNISTImages::Next()
{
    cv::Mat img;
    if(nb_items_ > 0) {

        img = cv::Mat(rows_, cols_, CV_8UC1);
        input_.read(reinterpret_cast<char*>(img.data), sizeof(unsigned char)*img.total());
    }
    nb_items_--;

    return img;
}

void ReadMNISTImages::Next(Signal &signal) {

    cv::Mat img(rows_, cols_, CV_8UC1);
    input_.read(reinterpret_cast<char*>(img.data), sizeof(unsigned char)*img.total());

    signal.Append(name_out_, static_cast<cv::Mat1f>(img));
}

#include <boost/assign/list_of.hpp>
template <>
elm::MapIONames LayerAttr_<ReadMNISTImagesTransl>::io_pairs = boost::assign::map_list_of
        ELM_ADD_OUTPUT_PAIR(ReadMNISTImagesTransl::KEY_OUTPUT);

ReadMNISTImagesTransl::ReadMNISTImagesTransl()
    : ReadMNISTImages(),
      scene_dims_(-1, -1)
{
}

bool ReadMNISTImagesTransl::IsSceneDimsSet() const
{
    return scene_dims_.width > 0 && scene_dims_.height > 0;
}

int ReadMNISTImagesTransl::ReadHeader(const string &path)
{
    int nb_items = ReadMNISTImages::ReadHeader(path);
    if(!IsSceneDimsSet()) {

        SceneDims(rows_, cols_);
    }
    else if(rows_*cols_ > scene_dims_.area()) {

        stringstream s;
        s << "Scene dims too small. Need min of ";
        s << "w(" << cols_ << ") h(" << rows_ << ")";
        ELM_THROW_BAD_DIMS(s.str());
    }
    return nb_items;
}

cv::Mat ReadMNISTImagesTransl::Next()
{
    cv::Mat mnist_img = ReadMNISTImages::Next();
    cv::Mat scene = cv::Mat::zeros(scene_dims_, mnist_img.type());

    current_loc_.x = static_cast<int>(cv::randu<uint32_t>() % (scene_dims_.width-mnist_img.cols));
    current_loc_.y = static_cast<int>(cv::randu<uint32_t>() % (scene_dims_.height-mnist_img.rows));

    mnist_img.copyTo(scene(cv::Rect2i(current_loc_, mnist_img.size())));
    return scene;
}

void ReadMNISTImagesTransl::SceneDims(int rows, int cols)
{
    if(rows > 0 && cols > 0) {

        scene_dims_ = cv::Size2i(cols, rows);
    }
    else {
        stringstream s;
        s << "Scene dims must be > 0. (" << rows << " rows, " << cols << " cols)";
        ELM_THROW_BAD_DIMS(s.str());
    }
}

cv::Rect2i ReadMNISTImagesTransl::Location() const
{
    return cv::Rect2i(current_loc_, cv::Size2i(cols_, rows_));
}


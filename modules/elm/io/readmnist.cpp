#include "elm/io/readmnist.h"

#include <iostream>
#include <sstream>

#include "elm/core/exception.h"
#include "elm/io/binary.h"

using namespace std;
using namespace elm;

base_ReadMNISTFile::base_ReadMNISTFile()
    : nb_items_(0)
{
}

int base_ReadMNISTFile::ReadHeader(const string &path)
{
    if(input_.is_open()) {

        input_.close();
    }

    input_.open(path, ios::in | ios::binary);

    if(!input_.is_open()) {

        ELM_THROW_FILEIO_ERROR("Failed to open file " + path);
    }

    int32_t tmp_i;

    input_.read(reinterpret_cast<char*>(&tmp_i), sizeof(int32_t));
    if(IS_32_LITTLE_ENDIAN) { SwapEndian<int32_t>(&tmp_i); }
    if(tmp_i != MagicNumber()) {

        stringstream s;
        s << "Unexpected magic number ("
          << tmp_i << "), expecting " << MagicNumber();
        ELM_THROW_FILEIO_ERROR(s.str());
    }

    input_.read(reinterpret_cast<char*>(&tmp_i), sizeof(int32_t));
    if(IS_32_LITTLE_ENDIAN) { SwapEndian<int32_t>(&tmp_i); }
    if(tmp_i < 0) {

        ELM_THROW_FILEIO_ERROR("No. of items must be >= 0");
    }
    nb_items_ = static_cast<int>(tmp_i);

    return nb_items_;
}

base_ReadMNISTFile::~base_ReadMNISTFile()
{
}

bool base_ReadMNISTFile::IS_EOF() const
{
    return nb_items_ <= 0;
}

ReadMNISTLabels::ReadMNISTLabels()
    : base_ReadMNISTFile()
{
}

int ReadMNISTLabels::MagicNumber() const {

    return MAGIC_NUMBER;
}

cv::Mat ReadMNISTLabels::Next()
{
    unsigned char next_label;
    if(nb_items_ > 0) {

        input_.read(reinterpret_cast<char*>(&next_label), sizeof(unsigned char));
    }
    nb_items_--;

    return cv::Mat1b(1, 1, next_label);
}

ReadMNISTImages::ReadMNISTImages()
    : base_ReadMNISTFile()
{
}

int ReadMNISTImages::ReadHeader(const string &path)
{
    base_ReadMNISTFile::ReadHeader(path);

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

    current_loc_.x = cv::randu<uint>() % (scene_dims_.width-mnist_img.cols);
    current_loc_.y = cv::randu<uint>() % (scene_dims_.height-mnist_img.rows);

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




#include "io/readmnist.h"

#include <iostream>
#include <sstream>
#include "core/exception.h"
#include "io/binary.h"

using namespace std;
using namespace sem;

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

        SEM_THROW_FILEIO_ERROR("Failed to open file " + path);
    }

    int32_t tmp_i;

    input_.read(reinterpret_cast<char*>(&tmp_i), sizeof(int32_t));
    if(IS_32_LITTLE_ENDIAN) { SwapEndian<int32_t>(&tmp_i); }
    if(tmp_i != MagicNumber()) {

        stringstream s;
        s << "Unexpected magic number ("
          << tmp_i << "), expecting " << MagicNumber();
        SEM_THROW_FILEIO_ERROR(s.str());
    }

    input_.read(reinterpret_cast<char*>(&tmp_i), sizeof(int32_t));
    if(IS_32_LITTLE_ENDIAN) { SwapEndian<int32_t>(&tmp_i); }
    if(tmp_i < 0) {

        SEM_THROW_FILEIO_ERROR("No. of items must be >= 0");
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

int ReadMNISTImages::MagicNumber() const {

    return MAGIC_NUMBER;
}

cv::Mat ReadMNISTImages::Next()
{
    unsigned char next_label;
    if(nb_items_ > 0) {

        input_.read(reinterpret_cast<char*>(&next_label), sizeof(unsigned char));
    }
    nb_items_--;

    return cv::Mat1b(1, 1, next_label);
}


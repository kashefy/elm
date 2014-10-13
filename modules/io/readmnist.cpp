#include "io/readmnist.h"

#include <iostream>
#include <sstream>
#include "core/exception.h"
#include "io/binary.h"

using namespace std;
using namespace sem;

base_ReadMNISTFile::base_ReadMNISTFile(const string &path)
    : nb_items_(0)
{
}

base_ReadMNISTFile::~base_ReadMNISTFile()
{
}

bool base_ReadMNISTFile::IS_EOF() const
{
    return nb_items_ <= 0;
}

ReadMNISTLabel::ReadMNISTLabel(const string &path)
    : base_ReadMNISTFile(path)
{
    input_.open(path, ios::in | ios::binary);

    if(!input_.is_open()) {

        SEM_THROW_FILEIO_ERROR("Failed to open file " + path);
    }

    int32_t tmp_i;

    input_.read(reinterpret_cast<char*>(&tmp_i), sizeof(int32_t));
    if(IS_32_LITTLE_ENDIAN) { SwapEndian<int32_t>(&tmp_i); }
    if(tmp_i != MAGIC_NUMBER) {

        stringstream s;
        s << "Unexpected magic number ("
          << tmp_i << "), expecting " << MAGIC_NUMBER;
        SEM_THROW_FILEIO_ERROR(s.str());
    }

    input_.read(reinterpret_cast<char*>(&tmp_i), sizeof(int32_t));
    if(IS_32_LITTLE_ENDIAN) { SwapEndian<int32_t>(&tmp_i); }
    if(tmp_i < 0) {

        SEM_THROW_FILEIO_ERROR("No. of items must be >= 0");
    }
    nb_items_ = static_cast<int>(tmp_i);
}

cv::Mat ReadMNISTLabel::Next()
{
    unsigned char next_label;
    if(nb_items_ > 0) {

        input_.read(reinterpret_cast<char*>(&next_label), sizeof(unsigned char));
    }
    nb_items_--;

    return cv::Mat1b(1, 1, next_label);
}

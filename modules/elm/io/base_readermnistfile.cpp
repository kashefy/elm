/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/io/base_readermnistfile.h"

#include "elm/core/exception.h"
#include "elm/core/layeroutputnames.h"
#include "elm/io/binary.h"

using namespace std;
using namespace elm;

const string base_ReaderMNISTFile::KEY_OUTPUT = detail::BASE_READERMNISTFILE_KEY_OUTPUT;

base_ReaderMNISTFile::~base_ReaderMNISTFile()
{
}

base_ReaderMNISTFile::base_ReaderMNISTFile()
    : base_Reader()
{
}

int base_ReaderMNISTFile::ReadHeader(const string &path)
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

void base_ReaderMNISTFile::OutputNames(const LayerOutputNames &io) {

    name_out_ = io.Output(KEY_OUTPUT);
}

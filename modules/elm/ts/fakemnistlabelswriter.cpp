/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/ts/fakemnistlabelswriter.h"

#include <sstream>

#include "elm/core/exception.h"
#include "elm/io/binary.h"              // determine and swap endianness

using namespace std;
using namespace elm;

FakeMNISTLabelsWriter::~FakeMNISTLabelsWriter()
{
}

FakeMNISTLabelsWriter::FakeMNISTLabelsWriter(const boost::filesystem::path& p)
    : path_(p)
{
}

void FakeMNISTLabelsWriter::Save(int magic_number)
{
    SaveHeader(magic_number);
    SaveItems();
    out_.close();
}

void FakeMNISTLabelsWriter::WriteInt(int n)
{
    if(IS_32_LITTLE_ENDIAN) { elm::SwapEndian(&n); }
    out_.write(reinterpret_cast<char*>(&n), sizeof(int32_t));
}

void FakeMNISTLabelsWriter::SaveHeader(int magic_number)
{
    if(out_.is_open()) {

        out_.close();
    }
    out_.open(path_.string().c_str(), ios::out | ios::binary);
    if(!out_.is_open()) {

        stringstream s;
        s << "Faied to write test file (" << path_ << ").";
        ELM_THROW_FILEIO_ERROR(s.str());
    }

    WriteInt(magic_number);
    WriteInt(NB_ITEMS);
}

void FakeMNISTLabelsWriter::SaveItems()
{
    for(int i=0; i<NB_ITEMS; i++) {

        unsigned char next_label = static_cast<unsigned char>(i%10);
        out_.write(reinterpret_cast<char*>(&next_label), sizeof(unsigned char));
    }
}

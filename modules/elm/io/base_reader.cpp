/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/io/base_reader.h"

#include <boost/filesystem.hpp>

#include "elm/core/exception.h"
#include "elm/core/layerconfig.h"

using std::string;
using std::stringstream;
namespace bfs=boost::filesystem;
using namespace elm;

const string base_Reader::PARAM_PATH = detail::BASE_READER_PARAM_PATH;

base_Reader::~base_Reader()
{
}

base_Reader::base_Reader()
    : base_Layer()
{
}

base_Reader::base_Reader(const LayerConfig &cfg)
    : base_Layer(cfg)
{
}

void base_Reader::Clear() {

    nb_items_ = 0;
    i_ = 0;
}

void base_Reader::Reconfigure(const LayerConfig& config) {

    PTree params = config.Params();

    bfs::path _path = params.get<bfs::path>(PARAM_PATH);

    if(!bfs::exists(_path)) {

        stringstream s;
        s << "Path does not exist (" << _path << ")";
        ELM_THROW_FILEIO_ERROR(s.str());
    }

    nb_items_ = ReadHeader(_path.string());

    if(nb_items_ < 1) {

        stringstream s;
        s << "No items found under (" << _path << ")";
        ELM_THROW_FILEIO_ERROR(s.str());
    }
}

void base_Reader::InputNames(const LayerInputNames& io) {

    // do nothing
}

void base_Reader::Activate(const Signal &signal) {

    // do nothing
}

void base_Reader::Response(Signal &signal) {

    if(i_ < nb_items_) {

        Next(signal);
        ++i_;
    }
}

bool base_Reader::Is_EOF() const {

    return i_ < nb_items_;
}


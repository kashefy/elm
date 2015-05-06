/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/io/readmnistlabels.h"

#include "elm/core/exception.h"
#include "elm/core/signal.h"
#include "elm/ts/layerattr_.h"

using namespace std;
using namespace elm;

#include <boost/assign/list_of.hpp>
template <>
elm::MapIONames LayerAttr_<ReadMNISTLabels>::io_pairs = boost::assign::map_list_of
        ELM_ADD_OUTPUT_PAIR(detail::BASE_READERMNISTFILE_KEY_OUTPUT);

ReadMNISTLabels::ReadMNISTLabels()
    : base_ReaderMNISTFile()
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

void ReadMNISTLabels::Next(Signal &signal) {

    unsigned char next_label;

    input_.read(reinterpret_cast<char*>(&next_label), sizeof(unsigned char));

    signal.Append(name_out_, cv::Mat1f(1, 1, static_cast<float>(next_label)));
}

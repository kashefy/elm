/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/io/file_serialization.h"

#include <boost/algorithm/string.hpp>

using namespace std;
using namespace elm;

detail::ArchiveType elm::detail::ArchiveTypeFromExt(const std::string &ext)
{
    using detail::ArchiveType;
    ArchiveType archive_type = ArchiveType::UNKNOWN;

    string ext_lower = boost::algorithm::to_lower_copy(ext);

    if(ext_lower == ".bin") {

        archive_type = BIN;
    }
    else if(ext_lower == ".txt" || ext_lower == ".text") {

        archive_type = TXT;
    }
    else if(ext_lower == ".xml") {

        archive_type = XML;
    }
    else {

        archive_type = UNKNOWN;
    }

    return archive_type;
}

/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/io/matlabmatfilereader.h"

#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

#include "matio.h"

#include "elm/core/exception.h"

using namespace std;
namespace bfs=boost::filesystem;
using namespace elm;

MatlabMATFileReader::MatlabMATFileReader()
    : matfp_(NULL)
{
}

MatlabMATFileReader::~MatlabMATFileReader()
{
    if(matfp_ != NULL) {

        Mat_Close(matfp_);
        matfp_ = NULL;
    }
}

void MatlabMATFileReader::ReadHeader(const string &path)
{
    bfs::path p(path);

    if(!bfs::is_regular_file(p)) {

        stringstream s;
        s << p.string();
        s << " is not a regular file.";
        ELM_THROW_FILEIO_ERROR(s.str());
    }

    string ext = p.extension().string();
    boost::algorithm::to_lower(ext);

    if(ext != ".mat") {

        stringstream s;
        s << p.string();
        s << " is not a .mat file.";
        ELM_THROW_FILEIO_ERROR(s.str());
    }

    if (matfp_ != NULL) {

        Mat_Close(matfp_);
    }

    matfp_ = Mat_Open(path.c_str(), MAT_ACC_RDONLY);

    if (matfp_ ==  NULL) {

        stringstream s;
        s << "Error opening MAT file ";
        s << p.string();
        ELM_THROW_FILEIO_ERROR(s.str());
    }

    matvar_t *var;
    while ((var = Mat_VarReadNextInfo(matfp_)) != NULL ) {

        string var_name(var->name);
        var_names_.push_back(var_name);

        Mat_VarFree(var);
        var = NULL;
    }
}

vector<string> MatlabMATFileReader::TopLevelVarNames() const
{
    return var_names_;
}

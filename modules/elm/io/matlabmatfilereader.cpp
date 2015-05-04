/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/io/matlabmatfilereader.h"

#include <boost/filesystem.hpp>         // path
#include <boost/algorithm/string.hpp>   // tolower()

#include <opencv2/core/core.hpp>        // convert to OpenCV Mat

#ifdef __WITH_MATIO

#include "matio.h"                      // io for matlab's .mat files

#include "elm/core/debug_utils.h"
#include "elm/core/exception.h"         // supported exceptions
#include "elm/io/matio_utils.h"

using namespace std;
namespace bfs=boost::filesystem;
using namespace cv;
using namespace elm;

MatlabMATFileReader::MatlabMATFileReader()
    : matfp_(NULL),
      cursor_(NULL)
{
}

MatlabMATFileReader::~MatlabMATFileReader()
{
    if(cursor_ != NULL) {

        Mat_VarFree(cursor_);
        cursor_ = NULL;
    }

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

        stringstream s;
        s << p.string();
        s << " is already been opened.";
        ELM_THROW_FILEIO_ERROR(s.str());
    }

    matfp_ = Mat_Open(path.c_str(), MAT_ACC_RDONLY);

    if (matfp_ ==  NULL) {

        stringstream s;
        s << "Error opening MAT file ";
        s << p.string();
        ELM_THROW_FILEIO_ERROR(s.str());
    }

    path_ = path;

    var_names_.clear();

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

void MatlabMATFileReader::Seek(const string &var_name)
{
    ResetFileHandle();

    cursor_ = Mat_VarReadInfo(matfp_, var_name.c_str());
    if ( NULL == cursor_ ) {

        stringstream s;
        s << "Variable \"" << var_name << "\" not found, or error reading MAT file";
        ELM_THROW_KEY_ERROR(s.str());
    }

}

matvar_t* MatlabMATFileReader::Cursor() const
{
    return cursor_;
}

Mat MatlabMATFileReader::CursorToMat() const
{
    Mat m;

    if(cursor_ == NULL) {

        ELM_THROW_KEY_ERROR("Cursor not pointing anywhere");
    }

    unsigned int cv_type = MATIOClassTOCV_TYPE(cursor_->class_type);

    int* sizes = new int[cursor_->rank];
    for(int i=0; i<cursor_->rank; i++) {

        sizes[i] = static_cast<int>(cursor_->dims[i]);
    }

    Mat_VarReadDataAll(matfp_, cursor_);

    m = Mat(cursor_->rank, sizes, CV_MAKE_TYPE(cv_type, 1), cursor_->data);

    delete []sizes;

    return m;
}

void MatlabMATFileReader::ResetFileHandle()
{
    if (matfp_ != NULL) {

        Mat_Rewind(matfp_);
    }
}

#endif // __WITH_MATIO


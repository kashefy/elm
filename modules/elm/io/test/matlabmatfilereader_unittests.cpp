/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/io/matlabmatfilereader.h"

#include <boost/filesystem.hpp>

#include "gtest/gtest.h"

#include "matio.h"

#include "elm/core/exception.h"

using namespace std;
namespace bfs=boost::filesystem; // use alias
using namespace elm;

class MatlabMATFileReaderTest : public ::testing::Test
{

};

TEST_F(MatlabMATFileReaderTest, MATIO)
{
    bfs::path p("/media/win/Users/woodstock/dev/data/nyu_depth_v2_labeled.mat");
    bool b = bfs::is_regular_file(p);

    int err = 0;
    mat_t *matfp;
    matvar_t *matvar;

    matfp = Mat_Open(p.string().c_str(), MAT_ACC_RDONLY);


    if ( NULL == matfp ) {

        fprintf(stderr,"Error opening MAT file");
    }
    while ( (matvar = Mat_VarReadNextInfo(matfp)) != NULL ) {

        printf("%s\n",matvar->name);


        Mat_VarFree(matvar);

        matvar = NULL;
    }
    Mat_Close(matfp);


//    mat_ft x = Mat_GetVersion(matfp);
//    matvar = Mat_VarReadInfo(matfp,"x");
//    matvar = Mat_VarReadInfo(matfp,"labels");
    //matvar=Mat_VarGetStructFieldByName(matvar,"numsample",1);
}

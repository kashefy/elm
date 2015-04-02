/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/io/matlabmatfilereader.h"

#include <fstream>

#include <boost/filesystem.hpp>

#include "gtest/gtest.h"

#ifdef __WITH_MATIO

#include "matio.h"

#include "elm/core/debug_utils.h"
#include "elm/core/exception.h"
#include "elm/ts/container.h"
#include "elm/ts/mat_assertions.h"

using namespace std;
namespace bfs=boost::filesystem; // use alias
using namespace cv;
using namespace elm;

namespace  {

class MatlabMATFileReaderTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        test_filepath_ = "test.mat";

        mat_t *matfp;
        matvar_t *matvar;

        const int ROWS=10;
        size_t dims[2] = {ROWS, 1};
        double x[ROWS] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10},
                y[ROWS] = {11,12,13,14,15,16,17,18,19,20};
        struct mat_complex_split_t z = {x, y};

        matfp = Mat_CreateVer(test_filepath_.c_str(), NULL, MAT_FT_DEFAULT);
        if ( NULL == matfp ) {

            fprintf(stderr,"Error creating MAT file \"test.mat\"\n");
        }

        matvar = Mat_VarCreate("x",
                               MAT_C_DOUBLE,
                               MAT_T_DOUBLE,
                               2,
                               dims,
                               x,
                               0);

        if ( NULL == matvar ) {

            fprintf(stderr,"Error creating variable for ’x’\n");
        }
        else {

            Mat_VarWrite(matfp,matvar, MAT_COMPRESSION_NONE);
            Mat_VarFree(matvar);
        }
        matvar = Mat_VarCreate("y",
                               MAT_C_DOUBLE,
                               MAT_T_DOUBLE,
                               2,
                               dims,
                               y,
                               0);

        if ( NULL == matvar ) {

            fprintf(stderr,"Error creating variable for ’y’\n");
        }
        else {

            Mat_VarWrite(matfp,matvar, MAT_COMPRESSION_NONE);
            Mat_VarFree(matvar);
        }
        matvar = Mat_VarCreate("z",
                               MAT_C_DOUBLE,
                               MAT_T_DOUBLE,
                               2,
                               dims,
                               &z,
                               MAT_F_COMPLEX);

        if ( NULL == matvar ) {

            fprintf(stderr,"Error creating variable for ’z’\n");
        }
        else {

            Mat_VarWrite(matfp,matvar, MAT_COMPRESSION_NONE);
            Mat_VarFree(matvar);
        }

        Mat_Close(matfp);
    }

    virtual void TearDown()
    {
        if(bfs::is_regular_file(test_filepath_)) {

            bfs::remove(test_filepath_);
        }
    }

    bfs::path test_filepath_;
};

TEST_F(MatlabMATFileReaderTest, Bad_path)
{
    MatlabMATFileReader reader;
    EXPECT_THROW(reader.ReadHeader(bfs::path("foo.mat").string()),
                 ExceptionFileIOError);
}

TEST_F(MatlabMATFileReaderTest, Bad_ext)
{
    string tmp_filepath("foo.txt");
    ofstream o(tmp_filepath.c_str());
    o << "bar";
    o.close();

    ASSERT_TRUE(bfs::is_regular_file(tmp_filepath));

    MatlabMATFileReader reader;
    EXPECT_THROW(reader.ReadHeader(tmp_filepath),
                 ExceptionFileIOError);

    bfs::remove(tmp_filepath);

    ASSERT_FALSE(bfs::is_regular_file(tmp_filepath));
}

TEST_F(MatlabMATFileReaderTest, TopLevelVarNames)
{
    bfs::path p(test_filepath_);

    MatlabMATFileReader to; ///< test object
    to.ReadHeader(p.string());

    vector<string> var_names = to.TopLevelVarNames();

    EXPECT_SIZE(3, var_names);
    EXPECT_EQ("x", var_names[0]);
    EXPECT_EQ("y", var_names[1]);
    EXPECT_EQ("z", var_names[2]);
}

TEST_F(MatlabMATFileReaderTest, Seek_invalid)
{
    bfs::path p(test_filepath_);

    MatlabMATFileReader to;
    to.ReadHeader(p.string());

    vector<string> var_names = to.TopLevelVarNames();

    EXPECT_SIZE(3, var_names);

    EXPECT_THROW(to.Seek("wrong"), ExceptionKeyError);
}

TEST_F(MatlabMATFileReaderTest, Cursor_init)
{
    MatlabMATFileReader to;
    ASSERT_TRUE(to.Cursor() == NULL);
}

TEST_F(MatlabMATFileReaderTest, Cursor)
{
    bfs::path p(test_filepath_);

    MatlabMATFileReader to;
    to.ReadHeader(p.string());

    EXPECT_THROW(to.Seek("wrong"), ExceptionKeyError);
    EXPECT_TRUE(to.Cursor() == NULL);

    to.Seek("x");
    EXPECT_FALSE(to.Cursor() == NULL);
}

TEST_F(MatlabMATFileReaderTest, CursorToMat_not_init)
{
    MatlabMATFileReader to;
    EXPECT_THROW(to.CursorToMat(), ExceptionKeyError);
}

TEST_F(MatlabMATFileReaderTest, CursorToMat_col_vector)
{
    bfs::path p(test_filepath_);

    MatlabMATFileReader to;
    to.ReadHeader(p.string());

    to.Seek("x");

    Mat x = to.CursorToMat();

    Mat1d expected(10, 1);
    for(int i=0; i<10; i++) {
        expected(i) = i+1;
    }
    EXPECT_MAT_DIMS_EQ(expected, x);
}

TEST_F(MatlabMATFileReaderTest, DISABLED_MATIO)
{
    bfs::path p("/media/win/Users/woodstock/dev/data/nyu_depth_v2_labeled.mat");

    MatlabMATFileReader to;
    to.ReadHeader(p.string());

    vector<string> var_names = to.TopLevelVarNames();
}

} // annonymous namespace for unit tests

#endif // __WITH_MATIO

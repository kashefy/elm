/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/io/matlabmatfilereader.h"

#ifdef __WITH_MATIO

#include <fstream>

#include <boost/filesystem.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "gtest/gtest.h"

#include "matio.h"

#include "elm/core/exception.h"
#include "elm/ts/container.h"
#include "elm/ts/mat_assertions.h"

#include "elm/io/matio_utils.h"

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

        double m[ROWS*2] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
                             11,12,13,14,15,16,17,18,19,20};

        size_t dims2[2] = {ROWS, 2};

        matvar = Mat_VarCreate("m",
                               MAT_C_DOUBLE,
                               MAT_T_DOUBLE,
                               2,
                               dims2,
                               &m,
                               0);

        if ( NULL == matvar ) {

            fprintf(stderr,"Error creating variable for ’m’\n");
        }
        else {

            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_NONE);
            Mat_VarFree(matvar);
        }

        double m3[4*3*2] = { 1, 2, 3,
                             4, 5, 6,
                             7, 8, 9,
                             10,11,12,

                             13,14,15,
                             16,17,18,
                             19,20,21,
                             22,23,24};

        size_t dims3[3] = {4, 3, 2};

        matvar = Mat_VarCreate("m3",
                               MAT_C_DOUBLE,
                               MAT_T_DOUBLE,
                               3,
                               dims3,
                               &m3,
                               0);

        if ( NULL == matvar ) {

            fprintf(stderr,"Error creating variable for ’m3’\n");
        }
        else {

            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_NONE);
            Mat_VarFree(matvar);
        }

        uint8_t m4[4*3*3*2] = {1, 2, 3,
                               4, 5, 6,
                               7, 8, 9,
                               10,11,12,

                               13,14,15,
                               16,17,18,
                               19,20,21,
                               22,23,24,

                               25,26,27,
                               28,29,30,
                               31,32,33,
                               34,35,36,

                               101,102,103,
                               104,105,106,
                               107,108,109,
                               110,111,112,

                               113,114,115,
                               116,117,118,
                               119,120,121,
                               122,123,124,

                               125, 126, 127,
                               128, 129, 130,
                               131, 132, 133,
                               134, 135, 136};

        size_t dims4[4] = {4, 3, 3, 2};

        matvar = Mat_VarCreate("m4",
                               MAT_C_UINT8,
                               MAT_T_UINT8,
                               4,
                               dims4,
                               &m4,
                               0);

        if ( NULL == matvar ) {

            fprintf(stderr,"Error creating variable for ’m4’\n");
        }
        else {

            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_NONE);
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

TEST_F(MatlabMATFileReaderTest, Bad_file)
{
    string tmp_filepath("foo.mat");
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

TEST_F(MatlabMATFileReaderTest, ReadHeader_twice)
{
    bfs::path p(test_filepath_);

    MatlabMATFileReader to; ///< test object
    EXPECT_NO_THROW(to.ReadHeader(p.string()));
    EXPECT_THROW(to.ReadHeader(p.string()), ExceptionFileIOError);

    vector<string> var_names = to.TopLevelVarNames();

    EXPECT_SIZE(6, var_names);
    EXPECT_EQ("x", var_names[3]);
    EXPECT_EQ("y", var_names[4]);
    EXPECT_EQ("z", var_names[5]);
}

TEST_F(MatlabMATFileReaderTest, TopLevelVarNames)
{
    bfs::path p(test_filepath_);

    MatlabMATFileReader to; ///< test object
    to.ReadHeader(p.string());

    vector<string> var_names = to.TopLevelVarNames();

    EXPECT_SIZE(6, var_names);
    EXPECT_EQ("x", var_names[3]);
    EXPECT_EQ("y", var_names[4]);
    EXPECT_EQ("z", var_names[5]);
}

TEST_F(MatlabMATFileReaderTest, Seek_invalid)
{
    bfs::path p(test_filepath_);

    MatlabMATFileReader to;
    to.ReadHeader(p.string());

    vector<string> var_names = to.TopLevelVarNames();

    EXPECT_SIZE(6, var_names);

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

TEST_F(MatlabMATFileReaderTest, CursorToMat_matrix)
{
    bfs::path p(test_filepath_);

    MatlabMATFileReader to;
    to.ReadHeader(p.string());

    to.Seek("m");

    Mat m = to.CursorToMat();

//    ELM_COUT_VAR(m);
//    Mat1d expected(10, 1);
//    for(int i=0; i<10; i++) {
//        expected(i) = i+1;
//    }
//    EXPECT_MAT_DIMS_EQ(expected, m);
}

TEST_F(MatlabMATFileReaderTest, Slice_3d)
{
    bfs::path p(test_filepath_);

    MatlabMATFileReader to;
    to.ReadHeader(p.string());

    to.Seek("m3");
    Mat x = to.CursorToMat();

    ASSERT_EQ(3, x.dims);

    Mat y;
    elm::SliceCopy(x, 2, 0, y);

    EXPECT_EQ(2, y.dims);

    EXPECT_MAT_DIMS_EQ(y, x.size());

    for(size_t i=0; i<y.total(); i++) {

        EXPECT_EQ(i+1, y.at<double>(i));
    }

    for(int r=0, v=1; r<y.rows; r++) {

        for(int c=0; c<y.cols; c++, v++) {

            EXPECT_EQ(v, y.at<double>(r, c));
        }
    }

    elm::SliceCopy(x, 2, 1, y);

    EXPECT_MAT_DIMS_EQ(y, x.size());

    for(size_t i=0; i<y.total(); i++) {

        EXPECT_EQ(i+13, y.at<double>(i));
    }

    for(int r=0, v=13; r<y.rows; r++) {

        for(int c=0; c<y.cols; c++, v++) {

            EXPECT_EQ(v, y.at<double>(r, c));
        }
    }
}

TEST_F(MatlabMATFileReaderTest, Mat3DToMat3Ch)
{
    bfs::path p(test_filepath_);

    MatlabMATFileReader to;
    to.ReadHeader(p.string());

    to.Seek("m4");
    Mat x = to.CursorToMat();

    ASSERT_EQ(4, x.dims);

    Mat y;
    elm::SliceCopy(x, 3, 0, y);

    Mat img;
    elm::Mat3DTo3Ch(y, img);

    EXPECT_EQ(2, img.dims);
    EXPECT_EQ(3, img.channels());
    EXPECT_EQ(x.size[2], img.channels());
    EXPECT_MAT_DIMS_EQ(img, cv::Size2i(x.size[1], x.size[0]));

//    ELM_COUT_VAR(static_cast<Mat3i>(img));

    img = img.t(); // easier to set expected values with tranposed image
    for(int r=0, i=1; r<img.rows; r++) {

        for(int c=0; c<img.cols; c++, i++) {

            Vec3b el = img.at<Vec3b>(r, c);

            EXPECT_EQ(i, static_cast<int>(el[0]));
            EXPECT_EQ(i+12, static_cast<int>(el[1]));
            EXPECT_EQ(i+12*2, static_cast<int>(el[2]));
        }
    }

    elm::SliceCopy(x, 3, 1, y);

    elm::Mat3DTo3Ch(y, img);

    img = img.t(); // easier to set expected values with tranposed image
    for(int r=0, i=101; r<img.rows; r++) {

        for(int c=0; c<img.cols; c++, i++) {

            Vec3b el = img.at<Vec3b>(r, c);

            EXPECT_EQ(i, static_cast<int>(el[0]));
            EXPECT_EQ(i+12, static_cast<int>(el[1]));
            EXPECT_EQ(i+12*2, static_cast<int>(el[2]));
        }
    }
}

TEST_F(MatlabMATFileReaderTest, DISABLED_MATIO)
{
    bfs::path p("/media/win/Users/woodstock/dev/data/nyu_depth_v2_labeled.mat");

    MatlabMATFileReader to;
    to.ReadHeader(p.string());

    vector<string> var_names = to.TopLevelVarNames();

    to.Seek("images");
    Mat x = to.CursorToMat();

    for(int i=0; i<1000; i++) {

        Mat y;
        elm::SliceCopy(x.clone(), 3, i, y);

        Mat img;
        elm::Mat3DTo3Ch(y, img);

        cv::cvtColor(img, img, CV_RGB2BGR);

        cv::imshow("image", img);
        cv::waitKey();
    }
}

} // annonymous namespace for unit tests

#endif // __WITH_MATIO

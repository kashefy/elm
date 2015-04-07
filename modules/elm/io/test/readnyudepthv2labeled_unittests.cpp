/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/io/readnyudepthv2labeled.h"

#include <boost/filesystem.hpp>

#include <opencv2/highgui/highgui.hpp>

#include "gtest/gtest.h"

#ifdef __WITH_MATIO

#include "elm/core/debug_utils.h"
#include "elm/core/exception.h"
#include "elm/ts/container.h"
#include "elm/ts/mat_assertions.h"
#include "elm/core/cv/mat_utils.h"

#include "elm/io/matio_utils.h"

using namespace std;
namespace bfs=boost::filesystem; // use alias
using namespace cv;
using namespace elm;

namespace  {

class ReadNYUDepthV2LabeledTest : public ::testing::Test
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

TEST_F(ReadNYUDepthV2LabeledTest, DISABLED_NYUV2)
{
    bfs::path p("/media/win/Users/woodstock/dev/data/nyu_depth_v2_labeled.mat");

    ReadNYUDepthV2Labeled to;
    int nb_items = to.ReadHeader(p.string());

    for(int i=0; i<10; i++) {

        ELM_COUT_VAR(i);

        cv::Mat bgr, labels, depth;
        to.Next(bgr, depth, labels);

        cv::imshow("image", bgr);
        cv::imshow("depth", elm::ConvertTo8U(depth));
        cv::imshow("labels", elm::ConvertTo8U(labels));
        cv::waitKey(0);
    }
}

} // annonymous namespace for unit tests

#endif // __WITH_MATIO

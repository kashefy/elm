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
        test_filepath_ = "fake_nyu_depth_v2.mat";

        mat_t *matfp;
        matvar_t *matvar;

        matfp = Mat_CreateVer(test_filepath_.c_str(), NULL, MAT_FT_DEFAULT);
        if ( NULL == matfp ) {

            fprintf(stderr,"Error creating MAT file \"fake_nyu_depth_v2.mat\"\n");
        }

        float m3[4*3*2] = { 1.1f, 2.2f, 3.3f,
                            4.4f, 5.5f, 6.6f,
                            7.7f, 8.8f, 9.9f,
                            10.f,11.1f,12.2f,

                            13.3f,14.4f,15.5f,
                            16.6f,17.7f,18.8f,
                            19.9f,20.0f,21.1f,
                            22.2f,23.3f,24.4f};

        size_t dims3[3] = {4, 3, 2};

        matvar = Mat_VarCreate("depths",
                               MAT_C_SINGLE,
                               MAT_T_SINGLE,
                               3,
                               dims3,
                               &m3,
                               0);

        if ( NULL == matvar ) {

            fprintf(stderr,"Error creating variable for ’depths’\n");
        }
        else {

            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_NONE);
            Mat_VarFree(matvar);
        }

        uint16_t labels[4*3*2] = { 1, 2, 3,
                                   4, 5, 6,
                                   7, 8, 9,
                                  10,11,12,

                                  13,14,15,
                                  16,17,18,
                                  19,20,21,
                                  22,23,24};

        matvar = Mat_VarCreate("labels",
                               MAT_C_SINGLE,
                               MAT_T_SINGLE,
                               3,
                               dims3,
                               &labels,
                               0);

        if ( NULL == matvar ) {

            fprintf(stderr,"Error creating variable for ’depths’\n");
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

        matvar = Mat_VarCreate("images",
                               MAT_C_UINT8,
                               MAT_T_UINT8,
                               4,
                               dims4,
                               &m4,
                               0);

        if ( NULL == matvar ) {

            fprintf(stderr,"Error creating variable for ’images’\n");
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

    void ReplaceVariableName(const bfs::path &src_path,
                             const std::string &src_name,
                             const bfs::path &dst_path,
                             const std::string &dst_name)
    {
        mat_t *matfp_src = Mat_Open(src_path.string().c_str(), MAT_ACC_RDWR);
        mat_t *matfp_dst = Mat_CreateVer(dst_path.string().c_str(), NULL, MAT_FT_DEFAULT);
        if ( NULL == matfp_dst ) {

            fprintf(stderr,"Error creating MAT file \"tmp.mat\"\n");
        }

        matvar_t *var;
        while ((var = Mat_VarReadNext(matfp_src)) != NULL ) {

            string var_name(var->name);
            if(var_name == src_name) {

                matvar_t *matvar2 = Mat_VarCreate(dst_name.c_str(),
                                                  var->class_type,
                                                  var->data_type,
                                                  var->rank,
                                                  var->dims,
                                                  var->data,
                                                  0);

                if ( NULL == matvar2 ) {

                    fprintf(stderr,"Error creating variable for ’foo’\n");
                }
                else {

                    Mat_VarWrite(matfp_dst, matvar2, MAT_COMPRESSION_NONE);
                    Mat_VarFree(matvar2);
                }
            }
            else {

                Mat_VarWrite(matfp_dst, var, MAT_COMPRESSION_NONE);
            }

            Mat_VarFree(var);
            var = NULL;
        }
        Mat_Close(matfp_src);
        Mat_Close(matfp_dst);
    }

    bfs::path test_filepath_;
};

TEST_F(ReadNYUDepthV2LabeledTest, Bad_Ext)
{
    ReadNYUDepthV2Labeled to;
    EXPECT_THROW(to.ReadHeader(bfs::path("foo.bar").string()), ExceptionFileIOError);
}

TEST_F(ReadNYUDepthV2LabeledTest, Bad_Path)
{
    ReadNYUDepthV2Labeled to;
    EXPECT_THROW(to.ReadHeader(bfs::path("foo.mat").string()), ExceptionFileIOError);
}

TEST_F(ReadNYUDepthV2LabeledTest, ReplaceVariableName_unchanged)
{
    bfs::path tmp_path("tmp.mat");

    ReplaceVariableName(test_filepath_, "depths", tmp_path, "depths");

    ReadNYUDepthV2Labeled to;
    EXPECT_NO_THROW(to.ReadHeader(tmp_path.string()));

    bfs::remove(tmp_path);
}

TEST_F(ReadNYUDepthV2LabeledTest, Bad_header_no_depths)
{
    {
        ReadNYUDepthV2Labeled to;
        EXPECT_NO_THROW(to.ReadHeader(test_filepath_.string()));
    }

    bfs::path tmp_path("tmp.mat");

    ReplaceVariableName(test_filepath_, "depths", tmp_path, "foo");

    ReadNYUDepthV2Labeled to;
    EXPECT_THROW(to.ReadHeader(tmp_path.string()), ExceptionKeyError);

    bfs::remove(tmp_path);
}

TEST_F(ReadNYUDepthV2LabeledTest, Bad_header_no_images)
{
    {
        ReadNYUDepthV2Labeled to;
        EXPECT_NO_THROW(to.ReadHeader(test_filepath_.string()));
    }

    bfs::path tmp_path("tmp.mat");

    ReplaceVariableName(test_filepath_, "images", tmp_path, "foo");

    ReadNYUDepthV2Labeled to;
    EXPECT_THROW(to.ReadHeader(tmp_path.string()), ExceptionKeyError);

    bfs::remove(tmp_path);
}

TEST_F(ReadNYUDepthV2LabeledTest, Bad_header_no_labels)
{
    {
        ReadNYUDepthV2Labeled to;
        EXPECT_NO_THROW(to.ReadHeader(test_filepath_.string()));
    }

    bfs::path tmp_path("tmp.mat");

    ReplaceVariableName(test_filepath_, "labels", tmp_path, "foo");

    ReadNYUDepthV2Labeled to;
    EXPECT_THROW(to.ReadHeader(tmp_path.string()), ExceptionKeyError);

    bfs::remove(tmp_path);
}

TEST_F(ReadNYUDepthV2LabeledTest, Nb_items)
{
    ReadNYUDepthV2Labeled to;
    EXPECT_EQ(2, to.ReadHeader(test_filepath_.string()));
}

TEST_F(ReadNYUDepthV2LabeledTest, Is_eof)
{
    ReadNYUDepthV2Labeled to;
    EXPECT_NO_THROW(to.ReadHeader(test_filepath_.string()));

    int count = 0;

    while(!to.IS_EOF()) {

        Mat bgr, depth, labels;
        to.Next(bgr, depth, labels);
        count++;
    }

    EXPECT_EQ(2, count);
}

TEST_F(ReadNYUDepthV2LabeledTest, Next_dims)
{
    ReadNYUDepthV2Labeled to;
    EXPECT_NO_THROW(to.ReadHeader(test_filepath_.string()));

    while(!to.IS_EOF()) {

        Mat bgr, depth, labels;
        to.Next(bgr, depth, labels);

        EXPECT_MAT_DIMS_EQ(bgr, Size2i(4, 3));
        EXPECT_EQ(3, bgr.channels());
        EXPECT_MAT_DIMS_EQ(depth, Size2i(3, 4));
        EXPECT_MAT_DIMS_EQ(labels, Size2i(3, 4));
    }
}

TEST_F(ReadNYUDepthV2LabeledTest, Next_first)
{
    ReadNYUDepthV2Labeled to;
    EXPECT_NO_THROW(to.ReadHeader(test_filepath_.string()));

    Mat bgr, depth, labels;
    to.Next(bgr, depth, labels);

    // inspect depth image
    {
        int i = 1;
        for(int c=0; c<depth.cols; c++) {

            for(int r=0; r<depth.rows; r++, i++) {

                float expected_value = static_cast<float>(i);
                expected_value += static_cast<float>(i % 10)/10.f;
                EXPECT_FLOAT_EQ(expected_value, depth.at<float>(r, c));
            }
        }
    }

    // inspect labels

    //ELM_COUT_VAR(labels);
    {
        int i = 1;
        for(int c=0; c<labels.cols; c++) {

            for(int r=0; r<labels.rows; r++, i++) {

                EXPECT_EQ(i, labels.at<int>(r, c));
            }
        }
    }

    // inspect bgr image
    //ELM_COUT_VAR(bgr);

    for(int r=0, i=1; r<bgr.rows; r++) {

        for(int c=0; c<bgr.cols; c++, i++) {

            Vec3b el = bgr.at<Vec3b>(r, c);

            // switch first and last channel because of bgr to rgb conversion
            EXPECT_EQ(i, static_cast<int>(el[2]));
            EXPECT_EQ(i+12, static_cast<int>(el[1]));
            EXPECT_EQ(i+12*2, static_cast<int>(el[0]));
        }
    }

}

TEST_F(ReadNYUDepthV2LabeledTest, DISABLED_NYUV2)
{
    bfs::path p("/media/win/Users/woodstock/dev/data/nyu_depth_v2_labeled.mat");

    ReadNYUDepthV2Labeled to;
    int nb_items = to.ReadHeader(p.string());

    ELM_COUT_VAR(nb_items);

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

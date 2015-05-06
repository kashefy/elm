/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/io/readmnistimages.h"

#include "elm/core/exception.h"
#include "elm/ts/fakemnistimageswriter.h"
#include "elm/ts/layer_assertions.h"    // custom assertions
#include "elm/ts/mat_assertions.h"      // custom assertions

using namespace std;
namespace bfs=boost::filesystem; // use alias
using namespace elm;

namespace {

ELM_INSTANTIATE_LAYER_TYPED_TEST_CASE_P(ReadMNISTImages);
ELM_INSTANTIATE_LAYER_TYPED_TEST_CASE_P(ReadMNISTImagesTransl);

class ReadMNISTImagesTest : public testing::Test
{
protected:
    static void SetUpTestCase()
    {
        bfs::create_directory(test_data_dir_);

        FakeMNISTImagesWriter writer(test_data_path_);
        writer.Save(ReadMNISTImages().MagicNumber());
    }

    static void TearDownTestCase()
    {
        if(bfs::is_directory(test_data_dir_)) {

            bfs::remove_all(test_data_dir_);
        }
    }

    static const bfs::path test_data_dir_;
    static const bfs::path test_data_path_;
};

const bfs::path ReadMNISTImagesTest::test_data_dir_("tmp_ReadMNISTFileTest");
const bfs::path ReadMNISTImagesTest::test_data_path_(test_data_dir_/"fake_data.tmp");

TEST_F(ReadMNISTImagesTest, WrongPath)
{
    EXPECT_THROW(ReadMNISTImages().ReadHeader("foo.bar"), ExceptionFileIOError);
}

TEST_F(ReadMNISTImagesTest, ReadHeader)
{
    const int N = FakeMNISTImagesWriter::NB_ITEMS;
    EXPECT_EQ(N, ReadMNISTImages().ReadHeader(test_data_path_.string().c_str()));
}

TEST_F(ReadMNISTImagesTest, Next)
{
    ReadMNISTImages to;
    const int N = to.ReadHeader(test_data_path_.string().c_str());
    const int ROWS = FakeMNISTImagesWriter::ROWS;
    const int COLS = FakeMNISTImagesWriter::COLS;

    EXPECT_GT(N, 0);

    for(int i=0; i<N*2; i++) {

        if(i<N) {

            EXPECT_FALSE(to.Is_EOF());
            cv::Mat img = to.Next();
            EXPECT_MAT_DIMS_EQ(img, cv::Mat(ROWS, COLS, CV_8UC1));
            int sum = cv::sum(img)(0);
            sum /= (ROWS*COLS);
            EXPECT_EQ(sum, static_cast<unsigned char>(i%255));

            if(i<N-1) { EXPECT_FALSE(to.Is_EOF()); }
            else { EXPECT_TRUE(to.Is_EOF()); }
        }
        else { EXPECT_TRUE(to.Is_EOF()); }
    }
}

TEST_F(ReadMNISTImagesTest, Invalid)
{
    bfs::path p = test_data_dir_/"fake_images_data_wrong_magic.tmp";
    FakeMNISTImagesWriter writer(p);
    ReadMNISTImages to;
    writer.Save(to.MagicNumber()+5);
    EXPECT_THROW(to.ReadHeader(p.string().c_str()), ExceptionFileIOError);
}

class ReadMNISTImagesTranslTest : public ReadMNISTImagesTest
{
protected:
    ReadMNISTImagesTransl to_; ///< test object
};

//class FakeMNISTImagesTranslWriter : public FakeMNISTImagesWriter
//{
//public:
//    FakeMNISTImagesTranslWriter(const bfs::path& p)
//        : FakeMNISTImagesWriter(p)
//    {
//        ;
//    }

//    virtual void SaveItems()
//    {

//    }
//};


TEST_F(ReadMNISTImagesTranslTest, WrongPath)
{
    EXPECT_THROW(ReadMNISTImagesTransl().ReadHeader("foo.bar"), ExceptionFileIOError);
}

TEST_F(ReadMNISTImagesTranslTest, SceneDims)
{
    EXPECT_THROW(ReadMNISTImagesTransl().SceneDims(1, 0), ExceptionBadDims);
    EXPECT_THROW(ReadMNISTImagesTransl().SceneDims(0, 1), ExceptionBadDims);
    EXPECT_THROW(ReadMNISTImagesTransl().SceneDims(-1, 1), ExceptionBadDims);
    EXPECT_THROW(ReadMNISTImagesTransl().SceneDims(1, -1), ExceptionBadDims);
    EXPECT_THROW(ReadMNISTImagesTransl().SceneDims(-1, -1), ExceptionBadDims);
    EXPECT_NO_THROW(ReadMNISTImagesTransl().SceneDims(1, 1));
}

TEST_F(ReadMNISTImagesTranslTest, ReadHeadersceneDims_too_small)
{
    ReadMNISTImagesTransl to; ///< test object

    EXPECT_NO_THROW(to.SceneDims(2, 2));
    EXPECT_THROW(to.ReadHeader(test_data_path_.string().c_str()), ExceptionBadDims);
}

TEST_F(ReadMNISTImagesTranslTest, ReadHeader)
{
    const int N = FakeMNISTImagesWriter::NB_ITEMS;
    EXPECT_EQ(N, ReadMNISTImagesTransl().ReadHeader(test_data_path_.string().c_str()));
}

TEST_F(ReadMNISTImagesTranslTest, Invalid)
{
    bfs::path p = test_data_dir_/"fake_images_data_wrong_magic.tmp";
    FakeMNISTImagesWriter writer(p);
    ReadMNISTImagesTransl to;
    writer.Save(to.MagicNumber()+5);
    EXPECT_THROW(to.ReadHeader(p.string().c_str()), ExceptionFileIOError);
}

TEST_F(ReadMNISTImagesTranslTest, EndOfFile)
{
    ReadMNISTImagesTransl to;
    to.SceneDims(20, 10);
    const int N = to.ReadHeader(test_data_path_.string().c_str());
    ASSERT_GT(N, 0);

    for(int i=0; i<N*2; i++) {

        if(i<N) {

            EXPECT_FALSE(to.Is_EOF());
            to.Next();
            if(i<N-1) { EXPECT_FALSE(to.Is_EOF()); }
            else { EXPECT_TRUE(to.Is_EOF()); }
        }
        else { EXPECT_TRUE(to.Is_EOF()); }
    }
}

TEST_F(ReadMNISTImagesTranslTest, NextAndLocation)
{
    ReadMNISTImagesTransl to;
    to.SceneDims(20, 10);
    const int N = to.ReadHeader(test_data_path_.string().c_str());
    ASSERT_GT(N, 0);
    const int ROWS = FakeMNISTImagesWriter::ROWS;
    const int COLS = FakeMNISTImagesWriter::COLS;

    for(int i=0; i<N*2; i++) {

        if(i<N) {

            EXPECT_FALSE(to.Is_EOF());
            cv::Mat scene_img = to.Next();
            EXPECT_MAT_DIMS_EQ(scene_img, cv::Size2i(10, 20));

            cv::Rect2i loc = to.Location();

            EXPECT_EQ(loc.width, COLS);
            EXPECT_EQ(loc.height, ROWS);
            cv::Rect scene_window(cv::Point2i(0, 0), scene_img.size());
            EXPECT_TRUE(scene_window.contains(loc.tl())) << "TL outside scene window." ;
            EXPECT_TRUE(scene_window.contains(loc.br())) << "BR outside scene window.";

            if(i<N-1) { EXPECT_FALSE(to.Is_EOF()); }
            else { EXPECT_TRUE(to.Is_EOF()); }
        }
        else { EXPECT_TRUE(to.Is_EOF()); }
    }
}

} // annonymous namespace for unit tests

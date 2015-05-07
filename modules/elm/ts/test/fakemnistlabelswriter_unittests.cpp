/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/ts/fakemnistlabelswriter.h"

#include "gtest/gtest.h"

#include "elm/core/exception.h"

namespace bfs=boost::filesystem;
using namespace elm;

namespace {

class FakeMNISTLabelsWriterTest : public ::testing::Test
{
protected:
    static void SetUpTestCase() {

        bfs::create_directory(test_data_dir_);
    }

    static void TearDownTestCase() {

        if(bfs::is_directory(test_data_dir_)) {

            bfs::remove_all(test_data_dir_);
        }
    }

    static const bfs::path test_data_dir_;
    static const bfs::path test_data_path_;
};
// initialize static members
const bfs::path FakeMNISTLabelsWriterTest::test_data_dir_("tmp_FakeMNISTLabelsWriterTest");
const bfs::path FakeMNISTLabelsWriterTest::test_data_path_(test_data_dir_/"fake_data.tmp");

TEST_F(FakeMNISTLabelsWriterTest, SaveHeader_invalid_parent_dir) {

    bfs::path wrong_dir("wrong_dir");
    wrong_dir = wrong_dir/test_data_path_;
    ASSERT_FALSE(bfs::is_directory(wrong_dir));
    FakeMNISTLabelsWriter writer(wrong_dir);

    EXPECT_THROW(writer.Save(2049), ExceptionFileIOError);
}

} // annonymous namespace for unit tests

/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/io/readmnist.h"

#include "elm/core/exception.h"
#include "elm/ts/fakemnistlabelswriter.h"
#include "elm/ts/layer_assertions.h"    // custom assertions
#include "elm/ts/mat_assertions.h"      // custom assertions

using namespace std;
namespace bfs=boost::filesystem; // use alias
using namespace elm;

namespace {

ELM_INSTANTIATE_LAYER_TYPED_TEST_CASE_P(ReadMNISTLabels);

/**
 * @brief class for testing ReadMNISTLabels
 * Fake data is written to disk first, then removed after last test
 */
class ReadMNISTLabelsTest : public testing::Test
{
protected:
    static void SetUpTestCase()
    {
        bfs::create_directory(test_data_dir_);
        FakeMNISTLabelsWriter writer(test_data_path_);

        writer.Save(ReadMNISTLabels().MagicNumber());
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
const bfs::path ReadMNISTLabelsTest::test_data_dir_("tmp_ReadMNISTFileTest");
const bfs::path ReadMNISTLabelsTest::test_data_path_(test_data_dir_/"fake_data.tmp");

TEST_F(ReadMNISTLabelsTest, WrongPath)
{
    EXPECT_THROW(ReadMNISTLabels().ReadHeader("foo.bar"), ExceptionFileIOError);
}

TEST_F(ReadMNISTLabelsTest, ReadHeader)
{
    const int N = FakeMNISTLabelsWriter::NB_ITEMS;
    EXPECT_EQ(N, ReadMNISTLabels().ReadHeader(test_data_path_.string().c_str()));
}

TEST_F(ReadMNISTLabelsTest, ReadHeader_repeated)
{
    const int N = FakeMNISTLabelsWriter::NB_ITEMS;
    EXPECT_EQ(N, ReadMNISTLabels().ReadHeader(test_data_path_.string().c_str()));
    EXPECT_EQ(N, ReadMNISTLabels().ReadHeader(test_data_path_.string().c_str()));
}

TEST_F(ReadMNISTLabelsTest, Next)
{
    ReadMNISTLabels to;
    const int N = to.ReadHeader(test_data_path_.string().c_str());

    EXPECT_GT(N, 0);

    for(int i=0; i<N*2; i++) {

        if(i<N) {

            EXPECT_FALSE(to.Is_EOF());
            int label = static_cast<int>(to.Next().at<unsigned char>(0));
            EXPECT_EQ(label, i%N);

            if(i<N-1) { EXPECT_FALSE(to.Is_EOF()); }
            else { EXPECT_TRUE(to.Is_EOF()); }
        }
        else { EXPECT_TRUE(to.Is_EOF()); }
    }
}

TEST_F(ReadMNISTLabelsTest, Invalid)
{
    bfs::path p = test_data_dir_/"fake_labels_data_wrong_magic.tmp";
    FakeMNISTLabelsWriter writer(p);
    ReadMNISTLabels to;
    writer.Save(to.MagicNumber()+5);
    EXPECT_THROW(to.ReadHeader(p.string().c_str()), ExceptionFileIOError);
}

} // anonymous namespace for unit tests

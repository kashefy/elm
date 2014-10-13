#include "io/readmnist.h"

#include "gtest/gtest.h"

#include <boost/filesystem.hpp>

#include "core/exception.h"
#include "io/binary.h"

using namespace std;
namespace bfs=boost::filesystem; // use alias

class FakeMNISTLabelWriter {

public:

    static const int NB_ITEMS = 10;

    FakeMNISTLabelWriter(const bfs::path& p)
        : path_(p)
    {
    }

    /**
     * @brief Save fake data to file
     * magic number
     * total no. of items
     * series label values [0,10)
     */
    void Save(int magic_number=ReadMNISTLabel::MAGIC_NUMBER) const
    {
        ofstream out;
        out.open(path_.string().c_str(), ios::out | ios::binary);
        if(!out.is_open()) {

            stringstream s;
            s << "Faied to write test file (" << path_ << ").";
            SEM_THROW_FILEIO_ERROR(s.str());
        }

        int32_t tmp_i;

        // magic number
        tmp_i = magic_number;
        if(IS_32_LITTLE_ENDIAN) sem::SwapEndian(&tmp_i);
        out.write(reinterpret_cast<char*>(&tmp_i), sizeof(int32_t));

        // no. of elements
        tmp_i = NB_ITEMS;
        if(IS_32_LITTLE_ENDIAN) { sem::SwapEndian(&tmp_i); }
        out.write(reinterpret_cast<char*>(&tmp_i), sizeof(int32_t));

        for(int i=0; i<NB_ITEMS; i++) {

            unsigned char next_label = static_cast<unsigned char>(i%10);
            out.write(reinterpret_cast<char*>(&next_label), sizeof(unsigned char));
        }
        out.close();
    }

protected:
    bfs::path path_;
};

class ReadMNISTLabelTest : public testing::Test
{
protected:
    static void SetUpTestCase()
    {
        bfs::create_directory(test_data_dir_);
        FakeMNISTLabelWriter writer(test_data_path_);
        writer.Save();
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
const bfs::path ReadMNISTLabelTest::test_data_dir_("tmp_ReadMNISTLabelTest");
const bfs::path ReadMNISTLabelTest::test_data_path_(test_data_dir_/"fake_label_data.tmp");

TEST_F(ReadMNISTLabelTest, WrongPath)
{
    EXPECT_THROW(ReadMNISTLabel("foo.bar"), ExceptionFileIOError);
}

TEST_F(ReadMNISTLabelTest, ReadFake)
{
    ReadMNISTLabel to(test_data_path_.string().c_str());

    const int N = FakeMNISTLabelWriter::NB_ITEMS;

    EXPECT_GT(N, 0);

    for(int i=0; i<N*2; i++) {

        if(i<N) {

            EXPECT_FALSE(to.IS_EOF());
            int label = static_cast<int>(to.Next().at<unsigned char>(0));
            EXPECT_EQ(label, i%N);

            if(i<N-1) { EXPECT_FALSE(to.IS_EOF()); }
            else { EXPECT_TRUE(to.IS_EOF()); }
        }
        else { EXPECT_TRUE(to.IS_EOF()); }
    }
}

TEST_F(ReadMNISTLabelTest, Invalid)
{
    bfs::path p = test_data_dir_/"fake_label_data_wrong_magic.tmp";
    FakeMNISTLabelWriter writer(p);
    writer.Save(ReadMNISTLabel::MAGIC_NUMBER+5);
    EXPECT_THROW(ReadMNISTLabel to(p.string().c_str()), ExceptionFileIOError);
}

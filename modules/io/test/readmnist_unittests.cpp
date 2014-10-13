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
    void Save(int magic_number) const
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

/**
 * @brief class for testing ReadMNISTLabel
 * Fake data is written to disk first, then removed after last test
 */
class ReadMNISTLabelTest : public testing::Test
{
protected:
    static void SetUpTestCase()
    {
        bfs::create_directory(test_data_dir_);
        FakeMNISTLabelWriter writer(test_data_path_);

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
const bfs::path ReadMNISTLabelTest::test_data_dir_("tmp_ReadMNISTLabelTest");
const bfs::path ReadMNISTLabelTest::test_data_path_(test_data_dir_/"fake_label_data.tmp");

TEST_F(ReadMNISTLabelTest, WrongPath)
{
    EXPECT_THROW(ReadMNISTLabels().ReadHeader("foo.bar"), ExceptionFileIOError);
}

TEST_F(ReadMNISTLabelTest, ReadHeader)
{
    const int N = FakeMNISTLabelWriter::NB_ITEMS;
    EXPECT_EQ(N, ReadMNISTLabels().ReadHeader(test_data_path_.string().c_str()));
}

TEST_F(ReadMNISTLabelTest, Next)
{
    ReadMNISTLabels to;
    const int N = to.ReadHeader(test_data_path_.string().c_str());

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
    ReadMNISTLabels to;
    writer.Save(to.MagicNumber()+5);
    EXPECT_THROW(to.ReadHeader(p.string().c_str()), ExceptionFileIOError);
}

/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/io/file_serialization.h"

#include "gtest/gtest.h"

#include "elm/core/exception.h"

using namespace elm;

namespace {

TEST(ArchiveTypeFromExtTest, ArchiveTypeFromExt)
{
    using namespace detail;
    EXPECT_EQ(ArchiveType::BIN, ArchiveTypeFromExt(".bin"));
    EXPECT_EQ(ArchiveType::TXT, ArchiveTypeFromExt(".txt"));
    EXPECT_EQ(ArchiveType::TXT, ArchiveTypeFromExt(".text"));
    EXPECT_EQ(ArchiveType::XML, ArchiveTypeFromExt(".xml"));
}

TEST(ArchiveTypeFromExtTest, ArchiveTypeFromExt_upper)
{
    using namespace detail;
    EXPECT_EQ(ArchiveType::BIN, ArchiveTypeFromExt(".BIN"));
    EXPECT_EQ(ArchiveType::TXT, ArchiveTypeFromExt(".TXT"));
    EXPECT_EQ(ArchiveType::TXT, ArchiveTypeFromExt(".TEXT"));
    EXPECT_EQ(ArchiveType::XML, ArchiveTypeFromExt(".XML"));
}

TEST(ArchiveTypeFromExtTest, ArchiveTypeFromExt_mixed_case)
{
    using namespace detail;
    EXPECT_EQ(ArchiveType::BIN, ArchiveTypeFromExt(".BiN"));
    EXPECT_EQ(ArchiveType::BIN, ArchiveTypeFromExt(".bIn"));
    EXPECT_EQ(ArchiveType::TXT, ArchiveTypeFromExt(".TxT"));
    EXPECT_EQ(ArchiveType::TXT, ArchiveTypeFromExt(".Text"));
    EXPECT_EQ(ArchiveType::XML, ArchiveTypeFromExt(".xmL"));
}

TEST(ArchiveTypeFromExtTest, ArchiveTypeFromExt_no_dot)
{
    using namespace detail;
    EXPECT_EQ(ArchiveType::UNKNOWN, ArchiveTypeFromExt("bin"));
    EXPECT_EQ(ArchiveType::UNKNOWN, ArchiveTypeFromExt("BIN"));
    EXPECT_EQ(ArchiveType::UNKNOWN, ArchiveTypeFromExt("txt"));
    EXPECT_EQ(ArchiveType::UNKNOWN, ArchiveTypeFromExt("TXT"));
    EXPECT_EQ(ArchiveType::UNKNOWN, ArchiveTypeFromExt("text"));
    EXPECT_EQ(ArchiveType::UNKNOWN, ArchiveTypeFromExt("xml"));
    EXPECT_EQ(ArchiveType::UNKNOWN, ArchiveTypeFromExt("XML"));
}

TEST(ArchiveTypeFromExtTest, Unknown)
{
    using namespace detail;
    EXPECT_EQ(ArchiveType::UNKNOWN, ArchiveTypeFromExt(".bin.swp"));
    EXPECT_EQ(ArchiveType::UNKNOWN, ArchiveTypeFromExt(".xml2"));
    EXPECT_EQ(ArchiveType::UNKNOWN, ArchiveTypeFromExt(".tx"));
    EXPECT_EQ(ArchiveType::UNKNOWN, ArchiveTypeFromExt("."));
}

class FileSerializationTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        fpath_ = boost::filesystem::path("foo.xml");
    }

    virtual void TearDown()
    {
        if(boost::filesystem::is_regular_file(fpath_)) {

            boost::filesystem::remove(fpath_);
        }
    }

    // members
    boost::filesystem::path fpath_;
};

TEST_F(FileSerializationTest, InvalidPath)
{
    int x = 5;

    boost::filesystem::path dir("bar");
    boost::filesystem::path fname("foo.xml");
    boost::filesystem::path fpath = dir/fname;

    ASSERT_FALSE(boost::filesystem::exists(dir));
    ASSERT_FALSE(boost::filesystem::exists(fpath));

    EXPECT_THROW(Load(fname, x), ExceptionFileIOError);
    EXPECT_THROW(Load(fpath, x), ExceptionFileIOError);
    EXPECT_THROW(Save(fpath, x), ExceptionFileIOError);
}

TEST_F(FileSerializationTest, InvalidExt)
{
    int x = 5;

    boost::filesystem::path fname("foo.doc");

    EXPECT_THROW(Load(fname, x), ExceptionValueError);
    EXPECT_THROW(Load(fname, x), ExceptionValueError);
    EXPECT_THROW(Save(fname, x), ExceptionValueError);
}

} // annonymous namespace for testing

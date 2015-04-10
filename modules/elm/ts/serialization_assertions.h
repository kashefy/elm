/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_TS_SERIALIZATION_ASSERTIONS_H_
#define _ELM_TS_SERIALIZATION_ASSERTIONS_H_

#include "gtest/gtest.h"

#include "elm/core/boost/serialization/serialization_utils.h"
#include "elm/io/file_serialization.h"

template <class T>
class SerializationTypedTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        dir_ = boost::filesystem::temp_directory_path() / "ser_tt";
        boost::filesystem::create_directory(dir_);

        ASSERT_TRUE(boost::filesystem::is_directory(dir_)) << "Cloud not create directory";
    }

    virtual void TearDown()
    {
        if(boost::filesystem::exists(dir_)) {

            boost::filesystem::remove_all(dir_);
        }
    }

    // members
    boost::filesystem::path dir_;
};

template<class T>
struct SerializationTypeAttr_
{
    static T example;
};

TYPED_TEST_CASE_P(SerializationTypedTest);

TYPED_TEST_P(SerializationTypedTest, Serialize_str_stream_bin)
{
    using namespace boost::archive;

    std::string str1;
    TypeParam obj;

    // serialize example to stream
    // then deserialize to other instance
    {
        std::stringstream stream;

        binary_oarchive oa1(stream);
        oa1 << SerializationTypeAttr_<TypeParam >::example;
        str1 = stream.str();

        binary_iarchive ia(stream);
        ia >> obj;
    }

    std::string str2;
    {
        std::stringstream stream;
        binary_oarchive oa(stream);
        oa << obj;
        str2 = stream.str();
    }

    EXPECT_EQ(str1, str2);
    EXPECT_NE(str1.c_str(), str2.c_str());
}

TYPED_TEST_P(SerializationTypedTest, Serialize_str_stream_txt)
{
    using namespace boost::archive;

    std::string str1;
    TypeParam obj;

    // serialize example to stream
    // then deserialize to other instance
    {
        std::stringstream stream;

        text_oarchive oa1(stream);
        oa1 << SerializationTypeAttr_<TypeParam >::example;
        str1 = stream.str();

        text_iarchive ia(stream);
        ia >> obj;
    }

    std::string str2;
    {
        std::stringstream stream;
        text_oarchive oa(stream);
        oa << obj;
        str2 = stream.str();
    }

    EXPECT_EQ(str1, str2);
    EXPECT_NE(str1.c_str(), str2.c_str());
}

TYPED_TEST_P(SerializationTypedTest, Serialize_str_stream_xml)
{
    using namespace boost::archive;

    std::string str1;
    TypeParam post_ser_obj;

    // serialize example to stream
    // then deserialize to other instance
    {
        std::stringstream stream;

        xml_oarchive oa1(stream);
        oa1 & boost::serialization::make_nvp(
                    "ser_example",
                    SerializationTypeAttr_<TypeParam >::example
                    );
        str1 = stream.str();

        xml_iarchive ia(stream);
        ia & BOOST_SERIALIZATION_NVP(post_ser_obj);
    }

    ASSERT_NE(str1.npos, str1.find_first_of("<ser_example "));

    std::string str2;
    {
        std::stringstream stream;
        xml_oarchive oa(stream);
        oa << BOOST_SERIALIZATION_NVP(post_ser_obj);
        str2 = stream.str();
    }

    std::string tag_open("<post_ser_obj ");

    ASSERT_NE(str2.npos, str2.find(tag_open));
    ASSERT_EQ(str2.npos, str2.find("<ser_example "));

    str2.replace(str2.find(tag_open),
                 tag_open.length(),
                 "<ser_example ");

    std::string tag_close("</post_ser_obj>");
    str2.replace(str2.find(tag_close),
                 tag_close.length(),
                 "</ser_example>");

    EXPECT_EQ(str1, str2);
    EXPECT_NE(str1.c_str(), str2.c_str());
}

TYPED_TEST_P(SerializationTypedTest, Serialize_str_stream_bin_wrapped)
{
    using namespace boost::archive;
    using namespace elm::detail;

    std::string str1;
    TypeParam obj;

    // serialize example to stream
    // then deserialize to other instance
    {
        std::stringstream stream;

        binary_oarchive oa1(stream);
        Save(oa1, SerializationTypeAttr_<TypeParam >::example);
        str1 = stream.str();

        binary_iarchive ia(stream);
        Load(ia, obj);
    }

    std::string str2;
    {
        std::stringstream stream;
        binary_oarchive oa(stream);
        Save(oa, obj);
        str2 = stream.str();
    }

    EXPECT_EQ(str1, str2);
    EXPECT_NE(str1.c_str(), str2.c_str());
}

TYPED_TEST_P(SerializationTypedTest, Serialize_str_stream_txt_wrapped)
{
    using namespace boost::archive;
    using namespace elm::detail;

    std::string str1;
    TypeParam obj;

    // serialize example to stream
    // then deserialize to other instance
    {
        std::stringstream stream;

        text_oarchive oa1(stream);
        Save(oa1, SerializationTypeAttr_<TypeParam >::example);
        str1 = stream.str();

        text_iarchive ia(stream);
        Load(ia, obj);
    }

    std::string str2;
    {
        std::stringstream stream;
        text_oarchive oa(stream);
        Save(oa, obj);
        str2 = stream.str();
    }

    EXPECT_EQ(str1, str2);
    EXPECT_NE(str1.c_str(), str2.c_str());
}

TYPED_TEST_P(SerializationTypedTest, Serialize_fstream_bin)
{
    using namespace boost::archive;

    std::string fpath = boost::filesystem::path(this->dir_/"foo.bin").string();

    std::string str1;
    TypeParam obj;

    // serialize example to stream
    // then deserialize to other instance
    {
        std::stringstream stream;
        binary_oarchive oa1(stream);
        oa1 << SerializationTypeAttr_<TypeParam >::example;
        str1 = stream.str();

        std::ofstream out(fpath.c_str());
        binary_oarchive oa2(out);
        oa2 << SerializationTypeAttr_<TypeParam >::example;
        out.close();

        std::ifstream in(fpath.c_str());
        binary_iarchive ia(in);
        ia >> obj;
    }

    std::string str2;
    {
        std::stringstream stream;
        binary_oarchive oa(stream);
        oa << obj;
        str2 = stream.str();
    }

    EXPECT_EQ(str1, str2);
    EXPECT_NE(str1.c_str(), str2.c_str());
}

TYPED_TEST_P(SerializationTypedTest, Serialize_fstream_txt)
{
    using namespace boost::archive;

    std::string fpath = boost::filesystem::path(this->dir_/"foo.txt").string();

    std::string str1;
    TypeParam obj;

    // serialize example to stream
    // then deserialize to other instance
    {
        std::stringstream stream;
        text_oarchive oa1(stream);
        oa1 << SerializationTypeAttr_<TypeParam >::example;
        str1 = stream.str();

        std::ofstream out(fpath.c_str());
        text_oarchive oa2(out);
        oa2 << SerializationTypeAttr_<TypeParam >::example;
        out.close();

        std::ifstream in(fpath.c_str());
        text_iarchive ia(in);
        ia >> obj;
    }

    std::string str2;
    {
        std::stringstream stream;
        text_oarchive oa(stream);
        oa << obj;
        str2 = stream.str();
    }

    EXPECT_EQ(str1, str2);
    EXPECT_NE(str1.c_str(), str2.c_str());
}

TYPED_TEST_P(SerializationTypedTest, Serialize_fstream_xml)
{
    using namespace boost::archive;

    std::string fpath = boost::filesystem::path(this->dir_/"foo.xml").string();

    std::string str1;
    TypeParam post_ser_obj;

    // serialize example to stream and file
    // then deserialize from file to other instance
    {
        std::ofstream out(fpath.c_str());

        xml_oarchive oa1(out);
        oa1 & boost::serialization::make_nvp(
                    "ser_example",
                    SerializationTypeAttr_<TypeParam >::example
                    );

        out.close();

        std::stringstream stream;

        xml_oarchive oa2(stream);
        oa2 & boost::serialization::make_nvp(
                    "ser_example",
                    SerializationTypeAttr_<TypeParam >::example
                    );
        str1 = stream.str();

        std::ifstream in(fpath.c_str());

        xml_iarchive ia(in);
        ia & BOOST_SERIALIZATION_NVP(post_ser_obj);
    }

    ASSERT_NE(str1.npos, str1.find_first_of("<ser_example "));

    std::string str2;
    {
        std::stringstream stream;
        xml_oarchive oa(stream);
        oa << BOOST_SERIALIZATION_NVP(post_ser_obj);
        str2 = stream.str();
    }

    std::string tag_open("<post_ser_obj ");

    ASSERT_NE(str2.npos, str2.find(tag_open));
    ASSERT_EQ(str2.npos, str2.find("<ser_example "));

    str2.replace(str2.find(tag_open),
                 tag_open.length(),
                 "<ser_example ");

    std::string tag_close("</post_ser_obj>");
    str2.replace(str2.find(tag_close),
                 tag_close.length(),
                 "</ser_example>");

    EXPECT_EQ(str1, str2);
    EXPECT_NE(str1.c_str(), str2.c_str());
}

TYPED_TEST_P(SerializationTypedTest, Serialize_fstream_bin_wrapped)
{
    using namespace boost::archive;

    std::string fpath = boost::filesystem::path(this->dir_/"foo.bin").string();

    std::string str1;
    TypeParam obj;

    // serialize example to stream
    // then deserialize to other instance
    {
        std::stringstream stream;
        binary_oarchive oa1(stream);
        oa1 << SerializationTypeAttr_<TypeParam >::example;
        str1 = stream.str();

        elm::Save(fpath, SerializationTypeAttr_<TypeParam >::example);
        elm::Load(fpath, obj);
    }

    std::string str2;
    {
        std::stringstream stream;
        binary_oarchive oa(stream);
        oa << obj;
        str2 = stream.str();
    }

    EXPECT_EQ(str1, str2);
    EXPECT_NE(str1.c_str(), str2.c_str());
}

TYPED_TEST_P(SerializationTypedTest, Serialize_fstream_txt_wrapped)
{
    using namespace boost::archive;

    std::string fpath = boost::filesystem::path(this->dir_/"foo.txt").string();

    std::string str1;
    TypeParam obj;

    // serialize example to stream
    // then deserialize to other instance
    {
        std::stringstream stream;
        text_oarchive oa1(stream);
        oa1 << SerializationTypeAttr_<TypeParam >::example;
        str1 = stream.str();

        elm::Save(fpath, SerializationTypeAttr_<TypeParam >::example);
        elm::Load(fpath, obj);
    }

    std::string str2;
    {
        std::stringstream stream;
        text_oarchive oa(stream);
        oa << obj;
        str2 = stream.str();
    }

    EXPECT_EQ(str1, str2);
    EXPECT_NE(str1.c_str(), str2.c_str());
}

TYPED_TEST_P(SerializationTypedTest, Serialize_fstream_xml_wrapped)
{
    using namespace boost::archive;

    std::string fpath = boost::filesystem::path(this->dir_/"foo.xml").string();

    std::string str1;
    TypeParam post_ser_obj;

    // serialize example to stream and file
    // then deserialize from file to other instance
    {
        std::stringstream stream;
        xml_oarchive oa2(stream);
        oa2 & boost::serialization::make_nvp(
                    "ser_example",
                    SerializationTypeAttr_<TypeParam >::example
                    );
        str1 = stream.str();

        TypeParam ser_example = SerializationTypeAttr_<TypeParam >::example;
        elm::Save(fpath, ser_example);
        elm::Load(fpath, post_ser_obj);
    }

    ASSERT_NE(str1.npos, str1.find_first_of("<ser_example "));

    std::string str2;
    {
        std::stringstream stream;
        xml_oarchive oa(stream);
        oa << BOOST_SERIALIZATION_NVP(post_ser_obj);
        str2 = stream.str();
    }

    std::string tag_open("<post_ser_obj ");

    ASSERT_NE(str2.npos, str2.find(tag_open));
    ASSERT_EQ(str2.npos, str2.find("<ser_example "));

    str2.replace(str2.find(tag_open),
                 tag_open.length(),
                 "<ser_example ");

    std::string tag_close("</post_ser_obj>");
    str2.replace(str2.find(tag_close),
                 tag_close.length(),
                 "</ser_example>");

    EXPECT_EQ(str1, str2);
    EXPECT_NE(str1.c_str(), str2.c_str());
}

/**
 * @brief serialize using text archive while Load() will assume binary archive.
 */
TYPED_TEST_P(SerializationTypedTest, Serialize_fstream_ext_matters_txt_bin)
{
    std::string fpath = boost::filesystem::path(this->dir_/"foo.bin").string();

    std::ofstream stream(fpath.c_str());
    boost::archive::text_oarchive oa1(stream);
    oa1 << SerializationTypeAttr_<TypeParam >::example;

    TypeParam obj;
    EXPECT_THROW(elm::Load(fpath, obj), boost::archive::archive_exception);
}

TYPED_TEST_P(SerializationTypedTest, Serialize_fstream_ext_matters_bin_txt)
{
    std::string fpath = boost::filesystem::path(this->dir_/"foo.txt").string();

    std::ofstream stream(fpath.c_str());
    boost::archive::binary_oarchive oa1(stream);
    oa1 << SerializationTypeAttr_<TypeParam >::example;

    TypeParam obj;
    EXPECT_THROW(elm::Load(fpath, obj), boost::archive::archive_exception);
}

// Register test names
REGISTER_TYPED_TEST_CASE_P(SerializationTypedTest,
                           Serialize_str_stream_bin,
                           Serialize_str_stream_txt,
                           Serialize_str_stream_xml,
                           Serialize_str_stream_bin_wrapped,
                           Serialize_str_stream_txt_wrapped,
                           Serialize_fstream_bin,
                           Serialize_fstream_txt,
                           Serialize_fstream_xml,
                           Serialize_fstream_bin_wrapped,
                           Serialize_fstream_txt_wrapped,
                           Serialize_fstream_xml_wrapped,
                           Serialize_fstream_ext_matters_txt_bin,
                           Serialize_fstream_ext_matters_bin_txt
                           ); ///< register additional typed_test_p (i.e. unit test) routines here


#endif // _ELM_TS_SERIALIZATION_ASSERTIONS_H_

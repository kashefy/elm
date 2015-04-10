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

#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>

#include "elm/core/boost/serialization/serialization_utils.h"

template <class T>
class SerializationTypedTest : public ::testing::Test
{
protected:
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

    std::string str1;
    TypeParam obj;

    // serialize example to stream
    // then deserialize to other instance
    {
        std::stringstream stream;

        binary_oarchive oa1(stream);
        elm::Save(oa1, SerializationTypeAttr_<TypeParam >::example);
        str1 = stream.str();

        binary_iarchive ia(stream);
        elm::Load(ia, obj);
    }

    std::string str2;
    {
        std::stringstream stream;
        binary_oarchive oa(stream);
        elm::Save(oa, obj);
        str2 = stream.str();
    }

    EXPECT_EQ(str1, str2);
    EXPECT_NE(str1.c_str(), str2.c_str());
}

TYPED_TEST_P(SerializationTypedTest, Serialize_str_stream_txt_wrapped)
{
    using namespace boost::archive;

    std::string str1;
    TypeParam obj;

    // serialize example to stream
    // then deserialize to other instance
    {
        std::stringstream stream;

        text_oarchive oa1(stream);
        elm::Save(oa1, SerializationTypeAttr_<TypeParam >::example);
        str1 = stream.str();

        text_iarchive ia(stream);
        elm::Load(ia, obj);
    }

    std::string str2;
    {
        std::stringstream stream;
        text_oarchive oa(stream);
        elm::Save(oa, obj);
        str2 = stream.str();
    }

    EXPECT_EQ(str1, str2);
    EXPECT_NE(str1.c_str(), str2.c_str());
}

// Register test names
REGISTER_TYPED_TEST_CASE_P(SerializationTypedTest,
                           Serialize_str_stream_bin,
                           Serialize_str_stream_txt,
                           Serialize_str_stream_xml,
                           Serialize_str_stream_bin_wrapped,
                           Serialize_str_stream_txt_wrapped
                           ); ///< register additional typed_test_p (i.e. unit test) routines here


#endif // _ELM_TS_SERIALIZATION_ASSERTIONS_H_

/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/core/boost/serialization/ser_mat.h"

#include "elm/ts/serialization_assertions.h"

using namespace std;
using cv::Mat;

namespace {

static Mat CreateExampleMat()
{
    Mat m(3, 2, CV_32FC1);

    for(size_t i=0; i<m.total(); i++) {

        m.at<float>(i) = i;
    }

    return m;
}

template<> Mat SerializationTypeAttr_<Mat>::example = CreateExampleMat();

//INSTANTIATE_TYPED_TEST_CASE_P(SerializationTypedTest_Mat_Test, SerializationTypedTest, Mat);

class SerializeMatTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        to_ = Mat(3, 2, CV_32FC1);

        for(size_t i=0; i<to_.total(); i++) {

            to_.at<float>(i) = i;
        }
    }

    Mat to_;    ///< test object
};

TEST_F(SerializeMatTest, Serialize2d)
{
    using namespace boost::archive;

    string str1;
    Mat obj;

    // serialize example to stream
    // then deserialize to other instance
    {
        stringstream stream;

        text_oarchive oa1(stream);
        oa1 << to_;
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

} // annonymous namespace for unit tests


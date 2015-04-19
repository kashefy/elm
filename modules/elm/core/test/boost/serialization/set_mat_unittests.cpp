/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/core/boost/serialization/ser_mat.h"

#include "elm/ts/mat_assertions.h"
#include "elm/ts/serialization_assertions.h"

using namespace std;
using namespace cv;

static Mat CreateExampleMat()
{
	Mat m(3, 2, CV_32FC1);

	for (size_t i = 0; i<m.total(); i++) {

		m.at<float>(i) = i;
	}

	return m;
}

template<> Mat SerializationTypeAttr_<Mat>::example = CreateExampleMat();

namespace {

INSTANTIATE_TYPED_TEST_CASE_P(SerializationTypedTest_Mat_Test, SerializationTypedTest, Mat);

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

TEST_F(SerializeMatTest, SerializeRowMatrix_float)
{
    using namespace boost::archive;

    string str1;
    Mat src(1, 10, CV_32FC1);

    for(size_t i=0; i<src.total(); i++) {

        src.at<float>(i) = static_cast<float>(i);
    }

    Mat obj;

    // serialize example to stream
    // then deserialize to other instance
    {
        stringstream stream;

        text_oarchive oa1(stream);
        oa1 << src;
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

    ASSERT_EQ(1, obj.rows) << "Result is not a row matrix";
    EXPECT_MAT_EQ(obj, src);
    EXPECT_EQ(obj.channels(), src.channels()) << "No. of channels do not match";
}

TEST_F(SerializeMatTest, SerializeColMatrix_float)
{
    using namespace boost::archive;

    string str1;
    Mat src(10, 1, CV_32FC1);

    for(size_t i=0; i<src.total(); i++) {

        src.at<float>(i) = static_cast<float>(i);
    }

    Mat obj;

    // serialize example to stream
    // then deserialize to other instance
    {
        stringstream stream;

        text_oarchive oa1(stream);
        oa1 << src;
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

    ASSERT_EQ(1, obj.cols) << "Result is not a row matrix";
    EXPECT_MAT_EQ(obj, src);
    EXPECT_EQ(obj.channels(), src.channels()) << "No. of channels do not match";
}

TEST_F(SerializeMatTest, Serialize3d)
{
    using namespace boost::archive;

    string str1;

    const int DIMS=3;
    const int SIZES[DIMS]={4, 2, 3};
    Mat src(DIMS, SIZES, CV_32FC1);

    for(size_t i=0; i<src.total(); i++) {

        src.at<float>(i) = static_cast<float>(i);
    }

    Mat obj;

    // serialize example to stream
    // then deserialize to other instance
    {
        stringstream stream;

        text_oarchive oa1(stream);
        oa1 << src;
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

    ASSERT_EQ(DIMS, obj.dims) << "Result is not a 3D matrix";
    EXPECT_EQ(obj.channels(), src.channels()) << "No. of channels do not match";

    for(int i=0; i<DIMS; i++) {

        EXPECT_EQ(SIZES[i], obj.size[i]) << "Dimension mismatch";
    }

    for(size_t i=0; i<src.total(); i++) {

        EXPECT_FLOAT_EQ(static_cast<float>(i), obj.at<float>(i));
    }
}

template<class T>
class SerializeMatTypedTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        to_ = Mat_<T>(3, 6);

        for(size_t i=0; i<to_.total(); i++) {

            to_(i) = static_cast<T>(i);
        }
    }

    // members
    Mat_<T> to_;    ///< test object
};

typedef ::testing::Types<float, int, uchar, double, uint8_t, uint16_t> PODTypes;
TYPED_TEST_CASE(SerializeMatTypedTest, PODTypes);

TYPED_TEST(SerializeMatTypedTest, Serialize2d)
{
    using namespace boost::archive;

    string str1;
    Mat src = static_cast<Mat>(this->to_);
    Mat obj;

    // serialize example to stream
    // then deserialize to other instance
    {
        stringstream stream;

        text_oarchive oa1(stream);
        oa1 << src;
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

    EXPECT_MAT_EQ(obj, src);
}

TYPED_TEST(SerializeMatTypedTest, Serialize2d2Ch)
{
    using namespace boost::archive;

    string str1;
    Mat src = static_cast<Mat>(static_cast<Mat_<Vec<TypeParam, 2> > >(this->to_));
    ASSERT_EQ(2, src.channels());
    Mat obj;

    // serialize example to stream
    // then deserialize to other instance
    {
        stringstream stream;

        text_oarchive oa1(stream);
        oa1 << src;
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

    EXPECT_EQ(2, obj.channels()) << "Expecting 2 channels.";
    EXPECT_EQ(src.channels(), obj.channels()) << "no. of channels do not match";
    EXPECT_MAT_DIMS_EQ(obj, src) << "Dimensions do not match";

    for(int r=0; r<src.rows; r++) {

        for(int c=0; c<src.cols; c++) {

            Vec<TypeParam, 2> el_src = src.at<Vec<TypeParam, 2> >(r, c);
            Vec<TypeParam, 2> el_obj = obj.at<Vec<TypeParam, 2> >(r, c);

            EXPECT_EQ(el_src[0], el_obj[0]);
            EXPECT_EQ(el_src[1], el_obj[1]);
            EXPECT_NE(el_src[1], el_obj[0]);
        }
    }
}

TYPED_TEST(SerializeMatTypedTest, Serialize2d3Ch)
{
    using namespace boost::archive;

    string str1;
    Mat src = static_cast<Mat>(static_cast<Mat_<Vec<TypeParam, 3> > >(this->to_));
    ASSERT_EQ(3, src.channels());
    Mat obj;

    // serialize example to stream
    // then deserialize to other instance
    {
        stringstream stream;

        text_oarchive oa1(stream);
        oa1 << src;
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

    EXPECT_EQ(3, obj.channels()) << "Expecting 2 channels.";
    EXPECT_EQ(src.channels(), obj.channels()) << "no. of channels do not match";
    EXPECT_MAT_DIMS_EQ(obj, src) << "Dimensions do not match";
    EXPECT_EQ(src.rows*src.cols, static_cast<int>(obj.total())) << "Unexpected no. of elements.";

    for(int r=0; r<src.rows; r++) {

        for(int c=0; c<src.cols; c++) {

            Vec<TypeParam, 3> el_src = src.at<Vec<TypeParam, 3> >(r, c);
            Vec<TypeParam, 3> el_obj = obj.at<Vec<TypeParam, 3> >(r, c);

            EXPECT_EQ(el_src[0], el_obj[0]);
            EXPECT_EQ(el_src[1], el_obj[1]);
            EXPECT_EQ(el_src[2], el_obj[2]);

            EXPECT_NE(el_src[1], el_obj[0]);
            EXPECT_NE(el_src[2], el_obj[0]);
            EXPECT_NE(el_src[2], el_obj[1]);
        }
    }
}

} // annonymous namespace for unit tests


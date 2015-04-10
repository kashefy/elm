/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/core/boost/serialization/ser_mat_utils.h"

#include "elm/ts/mat_assertions.h"
#include "elm/ts/serialization_assertions.h"

using namespace std;
using namespace cv;

namespace {

template<class T>
class SerializeMat2dTypedTest : public ::testing::Test
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
TYPED_TEST_CASE(SerializeMat2dTypedTest, PODTypes);

TYPED_TEST(SerializeMat2dTypedTest, Serialize2d_typed_mat)
{
    using namespace boost::archive;

    string str1;
    Mat_<TypeParam > src = this->to_;
    Mat_<TypeParam > obj;

    // serialize example to stream
    // then deserialize to other instance
    {
        stringstream stream;

        text_oarchive oa1(stream);
        elm::Save(oa1, src);
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

    EXPECT_EQ(obj.type(), src.type()) << "type mismatch";
    EXPECT_MAT_EQ(obj, src);
}

} // annonymous namespace for unit tests


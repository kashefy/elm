#include "io/readmnist.h"

#include "gtest/gtest.h"

#include "core/exception.h"

using namespace std;

TEST(ReadMNISTLabelTest, WrongPath)
{
    EXPECT_THROW(ReadMNISTLabel("foo.bar"), ExceptionFileIOError);
}

TEST(ReadMNISTLabelTest, Read)
{
    ReadMNISTLabel to("C:\\Users\\woodstock\\dev\\data\\MNIST\\t10k-labels.idx1-ubyte");
    for(int i=0; i<10001; i++) {

        std::cout<<static_cast<int>(to.Next())<<std::endl;
    }
}

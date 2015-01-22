#include "elm/io/binary.h"

#include "gtest/gtest.h"

#include "elm/core/exception.h"

using namespace std;

TEST(EndianTest, LittleXORBig)
{
    EXPECT_FALSE(O32_BIG_ENDIAN == O32_LITTLE_ENDIAN) << "Little can't be big";
    EXPECT_TRUE(((O32_HOST_ORDER == O32_LITTLE_ENDIAN) !=
                (O32_HOST_ORDER == O32_BIG_ENDIAN)) !=
                (O32_HOST_ORDER == O32_PDP_ENDIAN))
            << "Can only be one type of endian";
}

TEST(EndianTest, ExtraMacro)
{
    EXPECT_TRUE(((O32_HOST_ORDER == O32_LITTLE_ENDIAN) ==
                (IS_32_LITTLE_ENDIAN)) ==
                (IS_32_ENDIAN(O32_LITTLE_ENDIAN)))
            << "Non-identical macros";
}

TEST(EndianTest, SwapEndianness)
{
    int32_t x = 17301504, dst = 2049;
    int32_t backup = x;
    EXPECT_NE(x, dst);
    elm::SwapEndian<int32_t>(&x);
    EXPECT_EQ(dst, x);
    elm::SwapEndian<int32_t>(&x);
    EXPECT_EQ(backup, x);
}


/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/core/graph/vertexcache.h"

#include "gtest/gtest.h"

#include "elm/core/exception.h"

using namespace std;
using namespace elm;

namespace {

class VertexCacheTest : public ::testing::Test
{
protected:
    virtual void SetUp() {

        to_ = VertexCache();
        to_.reserve(10);

        to_.insert(3, VtxDescriptor());
        to_.insert(5, VtxDescriptor());
        to_.insert(2, VtxDescriptor());
    }

    VertexCache to_;    ///< test object
};

TEST_F(VertexCacheTest, Id)
{
    EXPECT_EQ(3, to_.Id(3));
    EXPECT_EQ(5, to_.Id(5));
    EXPECT_EQ(2, to_.Id(2));
}

TEST_F(VertexCacheTest, Id_invalid)
{
    EXPECT_THROW(to_.Id(-1), ExceptionKeyError);
    EXPECT_THROW(to_.Id(-100), ExceptionKeyError);
    EXPECT_THROW(to_.Id(1000000), ExceptionKeyError);
}

TEST_F(VertexCacheTest, Substitution_id)
{
    to_.recordSubstitution(3, 5);
    EXPECT_EQ(5, to_.Id(3));
    EXPECT_EQ(5, to_.Id(5));
    EXPECT_EQ(2, to_.Id(2));
}

TEST_F(VertexCacheTest, Substitution_id_chained)
{
    to_.recordSubstitution(3, 5);
    to_.recordSubstitution(5, 2);
    EXPECT_EQ(2, to_.Id(3));
    EXPECT_EQ(2, to_.Id(5));
    EXPECT_EQ(2, to_.Id(2));
}

TEST_F(VertexCacheTest, Substitution_isMasked)
{
    EXPECT_FALSE(to_.isMasked(3));
    EXPECT_FALSE(to_.isMasked(5));
    EXPECT_FALSE(to_.isMasked(2));

    to_.recordSubstitution(3, 5);

    EXPECT_FALSE(to_.isMasked(3));
    EXPECT_FALSE(to_.isMasked(5));
    EXPECT_FALSE(to_.isMasked(2));
}

TEST_F(VertexCacheTest, Substitution_isLink)
{
    EXPECT_FALSE(to_.isLink(3));
    EXPECT_FALSE(to_.isLink(5));
    EXPECT_FALSE(to_.isLink(2));

    to_.recordSubstitution(3, 5);

    EXPECT_TRUE(to_.isLink(3));

    EXPECT_FALSE(to_.isLink(5));
    EXPECT_FALSE(to_.isLink(2));

    // invalid ones too

    EXPECT_FALSE(to_.isLink(-1));
    EXPECT_FALSE(to_.isLink(-100));
    EXPECT_FALSE(to_.isLink(1000));
}

TEST_F(VertexCacheTest, Exists)
{
    EXPECT_TRUE(to_.exists(3));
    EXPECT_TRUE(to_.exists(5));
    EXPECT_TRUE(to_.exists(2));

    EXPECT_FALSE(to_.exists(-1));
    EXPECT_FALSE(to_.exists(-100));
    EXPECT_FALSE(to_.exists(10000));
}

TEST_F(VertexCacheTest, Exists_substituted)
{
    ASSERT_TRUE(to_.exists(3));
    ASSERT_TRUE(to_.exists(5));

    to_.recordSubstitution(3, 5);

    EXPECT_FALSE(to_.exists(3));
    EXPECT_TRUE(to_.exists(5));
}

TEST_F(VertexCacheTest, Remove)
{
    to_.remove(3);

    EXPECT_EQ(-1, to_.Id(3));
    EXPECT_FALSE(to_.exists(3));
}

TEST_F(VertexCacheTest, Remove_before_substitution)
{
    to_.remove(5);
    EXPECT_THROW(to_.recordSubstitution(3, 5), elm::ExceptionKeyError);
}

TEST_F(VertexCacheTest, Remove_with_substitution)
{
    to_.recordSubstitution(3, 5);
    to_.remove(5);

    EXPECT_FALSE(to_.exists(3));

    EXPECT_EQ(-1, to_.Id(3));

    EXPECT_FALSE(to_.exists(5));

    EXPECT_EQ(-1, to_.Id(5));
}

} // annonymous namespace for test cases

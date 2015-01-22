#include "sem/core/boost/ptree_utils.h"

#include "sem/core/exception.h"
#include "sem/core/stl/typedefs.h"
#include "sem/ts/ts.h"

using namespace std;
using namespace boost::property_tree;

using namespace elm;

/**
 * @brief test class around printing property trees
 */
class PTreeUtilsPrintTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        pt_ = PTree();
        pt_.put("k1", 1);
        pt_.put("k2", 2.1f);

        PTree p_sub;

        p_sub.put("sub1", "foo");
        p_sub.put("sub2", "bar");

        pt_.put_child("k4", p_sub);
    }

    // members
    PTree pt_;
};

TEST_F(PTreeUtilsPrintTest, DISABLED_PrintXML_Display) {

    PrintXML(pt_);
}

TEST_F(PTreeUtilsPrintTest, PrintXML) {

    stringstream s;
    PrintXML(pt_, s);
    EXPECT_EQ("<?xml version=\"1.0\" encoding=\"utf-8\"?>\n<k1>1</k1><k2>2.1</k2><k4><sub1>foo</sub1><sub2>bar</sub2></k4>",
              s.str());
}

TEST_F(PTreeUtilsPrintTest, Empty) {

    stringstream s;
    PrintXML(ptree(), s);
    EXPECT_EQ("<?xml version=\"1.0\" encoding=\"utf-8\"?>\n", s.str());
}

/**
 * @brief test class around routine for finding keys outside of a required set of keys
 */
class PTreeUtilsUnusedKeysTest : public PTreeUtilsPrintTest
{
};

TEST_F(PTreeUtilsUnusedKeysTest, Empty_PTree) {

    VecS used(1, "wrong"), unused;
    UnusedNodes(ptree(), used, unused);
    EXPECT_EMPTY(unused);
}

TEST_F(PTreeUtilsUnusedKeysTest, Empty_UsedKeys) {

    VecS used, unused;
    UnusedNodes(pt_, used, unused);

    EXPECT_EQ(VecS({"k1", "k2", "k4"}), unused) << "Not all keys accounted for.";
}

TEST_F(PTreeUtilsUnusedKeysTest, Empty_PTreeAndUsedKeys) {

    VecS used, unused;
    UnusedNodes(ptree(), used, unused);
    EXPECT_EQ(VecS(), unused);
}

TEST_F(PTreeUtilsUnusedKeysTest, UnusedKeys) {

    VecS used(1, "k1"), unused;
    UnusedNodes(pt_, used, unused);
    EXPECT_EQ(VecS({"k2", "k4"}), unused) << "Unexpected set of unused keys";
}

TEST_F(PTreeUtilsUnusedKeysTest, AllUsedKeys) {

    VecS used({"k1", "k4", "k2"}), unused;
    UnusedNodes(pt_, used, unused);
    EXPECT_EQ(VecS(), unused) << "Unexpected set of unused keys";
}

/**
 * @brief A setup for repeating tests with different types of PTree nodes (int, float, uchar)
 */
template <class T>
class PTreePODTypesTest : public testing::Test
{
};

/**
 * @brief the struct below enables defining values to be used inside the tests
 * These values are set below once per type.
 */
template<class T>
struct PTreeV_
{
    static std::vector<T> values;
};

TYPED_TEST_CASE_P(PTreePODTypesTest);

/** @brief tests around utility function for extracting multiple nodes into a vector
 *
 *  empty in empty out
 */
TYPED_TEST_P(PTreePODTypesTest, PTreePushBackChildTest_Empty_RootKey) {

    vector<TypeParam> v_out;
    push_back_child(PTree(), "", v_out);
    EXPECT_SIZE(0, v_out);
}

TYPED_TEST_P(PTreePODTypesTest, PTreePushBackChildTest_Empty_InvalidKey)
{
    vector<TypeParam> x;
    EXPECT_THROW(push_back_child(PTree(), "key", x), boost::property_tree::ptree_bad_path);
}

TYPED_TEST_P(PTreePODTypesTest, PTreePushBackChildTest_PushBack_Single)
{
    std::vector<TypeParam> v = PTreeV_<TypeParam>::values;

    ASSERT_GT(v.size(), size_t(0)) << "this test is useless if vector is empty.";

    for(size_t i=0; i<v.size(); i++) {

        PTree children, child, p;
        child.put("", v[i]);
        children.push_back(std::make_pair("", child));
        p.add_child("key", children);

        vector<TypeParam> x;
        push_back_child(p, "key", x);

        EXPECT_SIZE(1, x) << "Size mismatch";
        EXPECT_EQ(v[i], x[0]) << "Value mismatch";
    }
}

TYPED_TEST_P(PTreePODTypesTest, PTreePushBackChildTest_PushBack_Multiple)
{
    std::vector<TypeParam> v = PTreeV_<TypeParam>::values;

    ASSERT_GT(v.size(), size_t(1)) << "this test is useless if vector contains only one element";

    for(size_t i=1; i<v.size(); i++) {

        PTree children, p;

        for(size_t j=0; j<i; j++) {

            PTree child;
            child.put("", v[j]);
            children.push_back(std::make_pair("", child));
        }

        p.add_child("key", children);
        vector<TypeParam> x;
        push_back_child(p, "key", x);

        EXPECT_SIZE(i, x) << "Size mismatch";

        for(size_t j=0; j<i; j++) {

            EXPECT_EQ(v[j], x[j]) << "Value mismatch";
        }
    }
}

/* Write additional type+value parameterized tests here.
 * Acquaint yourself with the values passed to along with each type.
 */

// Register test names
REGISTER_TYPED_TEST_CASE_P(PTreePODTypesTest,
                           PTreePushBackChildTest_Empty_RootKey,
                           PTreePushBackChildTest_Empty_InvalidKey,
                           PTreePushBackChildTest_PushBack_Single,
                           PTreePushBackChildTest_PushBack_Multiple); ///< register additional typed_test_p (i.e. unit test) routines here

///< Register values to work with inside tests, note how they're used inside the tests
template<> std::vector<float> PTreeV_<float>::values{-1.f, 0.f, 1.f, 100.f, 101.f, 200.f};
template<> std::vector<int> PTreeV_<int>::values{-1, 0, 1, 100, 101, 200};
template<> std::vector<uchar> PTreeV_<uchar>::values{255, 0, 1, 100, 101, 200};

typedef testing::Types<float, int, uchar> PODTypes;  ///< // lists the usual suspects of matrices
INSTANTIATE_TYPED_TEST_CASE_P(PTreeUtilsPODTypesTest, PTreePODTypesTest, PODTypes);



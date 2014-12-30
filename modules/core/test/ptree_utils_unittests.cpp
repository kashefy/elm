#include "core/ptree_utils.h"

#include "gtest/gtest.h"

using namespace std;
using namespace boost::property_tree;

using namespace sem;

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

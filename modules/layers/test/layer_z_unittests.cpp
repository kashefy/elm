#include "layers/layer_z.h"

#include "ts/ts.h"

/**
 * @brief class for testing layer Z, the main, SEM learning algorithm
 */
class LayerZTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {

    }

    LayerZ to_; ///< test object
};

/**
 * @brief test hard reset
 */
TEST_F(LayerZTest, Reset)
{

}

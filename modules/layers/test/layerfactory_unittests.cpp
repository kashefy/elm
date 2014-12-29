#include "layers/layerfactory.h"

#include "core/exception.h"
#include "layers/saliencyitti.h" ///< to have layer dervied classes to test with
#include "layers/weightedsum.h"  ///< to have layer dervied classes to test with
#include "ts/ts.h"

using std::shared_ptr;

namespace {

/**
 * @brief class or testing LayerFactory's static methods
 */
class LayerFactoryStaticTest : public ::testing::Test
{
};

TEST_F(LayerFactoryStaticTest, CreateLayerPtrShared)
{
    {
        shared_ptr<base_Layer> ptr = LayerFactory::CreateLayerPtrShared("LayerZ");
        EXPECT_TRUE(bool(ptr));
    }
    {
        shared_ptr<base_Layer> ptr = LayerFactory::CreateLayerPtrShared("WeightedSum");
        EXPECT_TRUE(bool(ptr));
    }
}

TEST_F(LayerFactoryStaticTest, CreateLayerPtrShared_WrongType)
{
    EXPECT_THROW(LayerFactory::CreateLayerPtrShared("Blahbla"), sem::ExceptionTypeError);
}

TEST_F(LayerFactoryStaticTest, CreateLayerPtrShared_UniqueInstancesSameType)
{
    const std::string TYPE="WeightedSum";

    shared_ptr<base_Layer> ptr1 = LayerFactory::CreateLayerPtrShared(TYPE);
    shared_ptr<base_Layer> ptr2 = LayerFactory::CreateLayerPtrShared(TYPE);

    EXPECT_NE(ptr1, ptr2);
}

} // annonymous namespace

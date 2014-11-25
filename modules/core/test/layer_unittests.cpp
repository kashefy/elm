#include "core/base_Layer.h"

#include "gtest/gtest.h"

#include <memory>

#include "core/exception.h"

using std::unique_ptr;

class ChildLayer : public base_Layer
{
public:
    virtual void Reset() {}

    virtual void Stimulus() {}

    virtual void Apply() {}

    virtual void Response() {}

    ChildLayer() {}
};

class LayerTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        to_.reset(new ChildLayer());
    }

    unique_ptr<base_Layer> to_; ///< test object
};

TEST_F(LayerTest, Reset)
{
    EXPECT_THROW(to_->Reset(LayerConfig()), ExceptionNotImpl);
}

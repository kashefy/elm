#include "elm/core/base_Layer.h"

#include "gtest/gtest.h"

#include <memory>

#include "elm/core/exception.h"
#include "elm/core/layerconfig.h"
#include "elm/core/signal.h"

using namespace std;
using std::unique_ptr;
using namespace cv;
using namespace elm;

/** class for deriving from base Layer for test purposes
  */
class DummyChildLayer : public base_Layer
{
public:
    virtual void Clear() {}

    virtual void Reconfigure(const LayerConfig &config) {}

    virtual void IONames(const LayerIONames &config) {}

    virtual void Activate(const Signal &signal) {}

    virtual void Apply() {}

    virtual void Response(Signal &signal) {}

    DummyChildLayer() {}
};

class LayerTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        to_.reset(new DummyChildLayer());
    }

    unique_ptr<base_Layer> to_; ///< test object
};

TEST_F(LayerTest, Reset)
{
    EXPECT_THROW(to_->Reset(LayerConfig()), ExceptionNotImpl);
}

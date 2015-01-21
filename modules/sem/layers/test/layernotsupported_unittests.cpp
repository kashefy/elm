#include "sem/layers/layernotsupported.h"

#include "gtest/gtest.h"

#include "sem/core/exception.h"
#include "sem/core/layerconfig.h"
#include "sem/core/signal.h"
#include "sem/layers/layerfactory.h"

using std::string;
using sem::ExceptionNotImpl;

namespace {

/**
 * @brief use macro for defining a layer as not supported
 */
class DummyLayerNotSupported : public SEM_LAYER_NOT_SUPPORTED(DummyLayerNotSupported, "Boooh");

/** @brief test routines around "dummy not supported layer".
 * Bbascially testing that Exceptions are thrown correctly
 */
TEST(LayerNotSupportedTest, ConstructorsThrow)
{
    EXPECT_THROW(DummyLayerNotSupported to,      ExceptionNotImpl);

    LayerConfig cfg;
    EXPECT_THROW(DummyLayerNotSupported to(cfg), ExceptionNotImpl);

    std::shared_ptr<base_LayerNotSupported> to_ptr;
    EXPECT_THROW(to_ptr.reset(new DummyLayerNotSupported()),    ExceptionNotImpl);
    EXPECT_THROW(to_ptr.reset(new DummyLayerNotSupported(cfg)), ExceptionNotImpl);
}

TEST(LayerNotSupportedTest, Message)
{
    try {
        DummyLayerNotSupported to;

    } catch(const ExceptionNotImpl &e) {

        std::string msg(e.what());
        EXPECT_NE(msg.find("Boooh"), msg.npos);
    }
}

class CheatLayerNotSupported : public base_LayerNotSupported
{
public:
    CheatLayerNotSupported(const string message=std::string("Boohoo"))
        : base_LayerNotSupported()
    {
        msg_ = message;
    }

    CheatLayerNotSupported(const LayerConfig &config, const string message=string("Boohoo"))
        : base_LayerNotSupported()
    {
        msg_ = message;
    }
};

TEST(LayerNotSupportedTest, Methods)
{
    LayerConfig cfg;
    LayerIONames io;
    Signal sig;

    CheatLayerNotSupported to; // test object
    EXPECT_THROW( to.Activate(sig),      ExceptionNotImpl );
    EXPECT_THROW( to.Clear(),            ExceptionNotImpl );
    EXPECT_THROW( to.IONames(io),        ExceptionNotImpl );
    EXPECT_THROW( to.Reconfigure(cfg),   ExceptionNotImpl );
    EXPECT_THROW( to.Reset(cfg),         ExceptionNotImpl );
    EXPECT_THROW( to.Response(sig),      ExceptionNotImpl );
}

} // anonymous namespace for test routines

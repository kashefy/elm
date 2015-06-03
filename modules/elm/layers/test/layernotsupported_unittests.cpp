/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/layers/layernotsupported.h"

#include "gtest/gtest.h"

#include "elm/core/exception.h"
#include "elm/core/layerconfig.h"
#include "elm/core/signal.h"
#include "elm/layers/layerfactory.h"

using std::string;
using namespace elm;

namespace {

/**
 * @brief use macro for defining a layer as not supported
 */
class DummyLayerNotSupported : public ELM_LAYER_NOT_SUPPORTED(DummyLayerNotSupported, "Boooh");

/** @brief test routines around "dummy not supported layer".
 * Bbascially testing that Exceptions are thrown correctly
 */
TEST(LayerNotSupportedTest, ConstructorsThrow)
{
    {
        EXPECT_THROW(DummyLayerNotSupported to,      ExceptionNotImpl);
    }
    {
        std::shared_ptr<base_LayerNotSupported> to_ptr;
        EXPECT_THROW(to_ptr.reset(new DummyLayerNotSupported()), ExceptionNotImpl);
    }
    {
        LayerShared to_ptr;
        EXPECT_THROW(to_ptr.reset(new DummyLayerNotSupported()), ExceptionNotImpl);
    }
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
    EXPECT_THROW( to.OutputNames(io),        ExceptionNotImpl );
    EXPECT_THROW( to.InputNames(io),        ExceptionNotImpl );
    EXPECT_THROW( to.Reconfigure(cfg),   ExceptionNotImpl );
    EXPECT_THROW( to.Reset(cfg),         ExceptionNotImpl );
    EXPECT_THROW( to.Response(sig),      ExceptionNotImpl );
}

/**
 * @brief use macro for defining a layer as not supported
 */
class DummyLayerNotSupportedNoMsg : public ELM_LAYER_NOT_SUPPORTED(DummyLayerNotSupportedNoMsg, "");

/** @brief test routines around "dummy not supported layer".
 * Bbascially testing that Exceptions are thrown correctly
 */
TEST(LayerNotSupportedNoMsgTest, ConstructorsThrow)
{
    {
        EXPECT_THROW(DummyLayerNotSupportedNoMsg to,      ExceptionNotImpl);
    }
    {
        std::shared_ptr<base_LayerNotSupported> to_ptr;
        EXPECT_THROW(to_ptr.reset(new DummyLayerNotSupportedNoMsg()),    ExceptionNotImpl);
    }
    {
        LayerShared to_ptr;
        EXPECT_THROW(to_ptr.reset(new DummyLayerNotSupportedNoMsg()),    ExceptionNotImpl);
    }
}

} // anonymous namespace for test routines

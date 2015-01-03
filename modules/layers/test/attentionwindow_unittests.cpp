#include "layers/attentionwindow.h"

#include "core/exception.h"
#include "core/signal.h"
#include "layers/layerfactory.h"
#include "ts/ts.h"

namespace bpt=boost::property_tree;
using namespace std;
using namespace cv;

namespace {

const string NAME_SCENE = "in";
const string NAME_LOC   = "loc";
const string NAME_TL    = "tl";
const string NAME_WIN   = "win";

/**
 * @brief testing fixture class around AttentionWindow layer initiaization
 */
class AttentionWindowInitTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        params_.put(AttentionWindow::PARAM_WIN_ROWS, 3);
        params_.put(AttentionWindow::PARAM_WIN_COLS, 4);

        cfg_.Params(params_);

        io_.Input(AttentionWindow::KEY_INPUT_LOC, NAME_LOC);
        io_.Input(AttentionWindow::KEY_INPUT_SCENE, NAME_SCENE);
        io_.Output(AttentionWindow::KEY_OUTPUT_WIN, NAME_WIN);
    }

    virtual void TearDown()
    {
        params_.clear();
    }

    //members
    shared_ptr<base_Layer> to_; ///< test object
    LayerConfig cfg_;   ///< default layer configuration
    LayerIONames io_;   ///< default i/o names
    PTree params_;      ///< default params
};

TEST_F(AttentionWindowInitTest, MissingParams)
{
    EXPECT_THROW(AttentionWindow().Reset(LayerConfig()), bpt::ptree_bad_path);

    {
        PTree p(params_);
        p.erase(AttentionWindow::PARAM_WIN_COLS);
        cfg_.Params(p);
        EXPECT_THROW(AttentionWindow().Reset(cfg_), bpt::ptree_bad_path);
    }

    {
        PTree p(params_);
        p.erase(AttentionWindow::PARAM_WIN_ROWS);
        cfg_.Params(p);
        EXPECT_THROW(AttentionWindow().Reset(cfg_), bpt::ptree_bad_path);
    }
}

TEST_F(AttentionWindowInitTest, Reset_ParamsPresent)
{
    EXPECT_NO_THROW(AttentionWindow().Reset(cfg_)) << "Any required paramters missing?";
}

TEST_F(AttentionWindowInitTest, IONames)
{
    LayerIONames io_names;
    EXPECT_THROW(AttentionWindow().IONames(io_names), std::exception);

    io_names.Input(AttentionWindow::KEY_INPUT_SCENE, NAME_SCENE);
    EXPECT_THROW(AttentionWindow().IONames(io_names), std::exception) << "still missing required io names";

    io_names.Input(AttentionWindow::KEY_INPUT_LOC, NAME_LOC);
    EXPECT_THROW(AttentionWindow().IONames(io_names), std::exception) << "still missing required outputs";

    io_names.Output(AttentionWindow::KEY_OUTPUT_WIN, NAME_WIN);
    EXPECT_NO_THROW(AttentionWindow().IONames(io_names)) << "Are all required IO names present?";
}

/**
 * @brief testing fixture class around AttentionWindow's 'live' layer methods
 * Assuming a fully and sucessfully configured layer instance
 */
class AttentionWindowTest : public AttentionWindowInitTest
{
protected:
    virtual void SetUp()
    {
        AttentionWindowInitTest::SetUp();

        io_.Output(AttentionWindow::KEY_OUTPUT_OPT_TL, NAME_TL);

        to_ = LayerFactory::CreateLayerPtrShared("AttentionWindow", cfg_, io_);
    }
};

} // anonnymous namespace around test fixtures



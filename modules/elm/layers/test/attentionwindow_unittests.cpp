/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/layers/attentionwindow.h"

#include "elm/core/exception.h"
#include "elm/core/cv/mat_utils.h"
#include "elm/core/signal.h"
#include "elm/layers/layerfactory.h"
#include "elm/ts/layer_assertions.h"
#include "elm/ts/mat_assertions.h"

namespace bpt=boost::property_tree;
using namespace std;
using namespace cv;
using namespace elm;

namespace {

const string NAME_SCENE = "in";
const string NAME_LOC   = "loc";
const string NAME_TL    = "tl";
const string NAME_WIN   = "win";

ELM_INSTANTIATE_LAYER_TYPED_TEST_CASE_P(AttentionWindow);

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

TEST_F(AttentionWindowInitTest, MissingParams_Reset)
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

TEST_F(AttentionWindowInitTest, Constructor)
{
    EXPECT_NO_THROW(AttentionWindow(cfg_)) << "Any required paramters missing?";
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

        to_ = LayerFactory::CreateShared("AttentionWindow", cfg_, io_);

        // append stimulus
        Mat1f stimulus(6, 8);
        randn(stimulus, 0.f, 100.f);
        sig_.Append(NAME_SCENE, stimulus);
        sig_.Append(NAME_LOC, static_cast<Mat1f>(elm::Point2Mat(Point2i(3, 4))));
    }

    virtual void TearDown()
    {
        AttentionWindowInitTest::TearDown();
        sig_.Clear();
    }

    /**
     * @brief Calculate centre coordinates given coordinates of top-left corner and rectangle size
     *
     * Fractions are ceiled.
     *
     * @param top-left corner coordinates
     * @param rectangle size
     * @return rectangle centre coordinates
     */
    Point2i TL2Centre(const Point2i &tl, const Size2i &sz) {

        Point2i centre = tl;
        centre.x += static_cast<int>(ceil(sz.width/2.));
        centre.y += static_cast<int>(ceil(sz.height/2.));
        return centre;
    }

    // members
    Signal sig_;
};

TEST_F(AttentionWindowTest, OptionalOutput)
{
    LayerIONames io;
    io.Input(AttentionWindow::KEY_INPUT_LOC, NAME_LOC);
    io.Input(AttentionWindow::KEY_INPUT_SCENE, NAME_SCENE);
    io.Output(AttentionWindow::KEY_OUTPUT_WIN, NAME_WIN);

    AttentionWindow to;
    to.Reset(cfg_);
    EXPECT_NO_THROW(to.IONames(io)) << "Are all required IO names present?";

    to.Activate(sig_);
    {
        Signal signal_new;
        to.Response(signal_new);
        EXPECT_TRUE(signal_new.Exists(NAME_WIN));
        EXPECT_FALSE(signal_new.Exists(NAME_TL));
    }

    io.Output(AttentionWindow::KEY_OUTPUT_OPT_TL, NAME_TL);
    EXPECT_NO_THROW(to.IONames(io)) << "Are all required IO names present?";

    to.Activate(sig_);
    {
        Signal signal_new;
        to.Response(signal_new);
        EXPECT_TRUE(signal_new.Exists(NAME_WIN));
        EXPECT_TRUE(signal_new.Exists(NAME_TL));
    }
}

TEST_F(AttentionWindowTest, MissingInputFeature)
{
    Signal s;
    EXPECT_THROW(to_->Activate(s), elm::ExceptionKeyError);

    s.Append(NAME_SCENE, sig_.MostRecentMat1f(NAME_SCENE));
    EXPECT_THROW(to_->Activate(s), elm::ExceptionKeyError);

    EXPECT_NO_THROW(to_->Activate(sig_)) << "Still missing input features?";
}

TEST_F(AttentionWindowTest, Response)
{
    EXPECT_FALSE(sig_.Exists(NAME_TL));
    EXPECT_FALSE(sig_.Exists(NAME_WIN));
    to_->Activate(sig_);
    to_->Response(sig_);
    EXPECT_TRUE(sig_.Exists(NAME_TL));
    EXPECT_TRUE(sig_.Exists(NAME_WIN));
}

TEST_F(AttentionWindowTest, WinSize)
{
    to_->Activate(sig_);
    to_->Response(sig_);
    EXPECT_TRUE(sig_.Exists(NAME_TL));
    EXPECT_TRUE(sig_.Exists(NAME_WIN));

    EXPECT_MAT_DIMS_EQ(sig_.MostRecentMat1f(NAME_WIN), Size2i(4, 3));
}

TEST_F(AttentionWindowTest, Window)
{
    const int N=10;
    const Mat1f scene = sig_.MostRecentMat1f(NAME_SCENE);
    for(int i=0; i<N; i++) {

        Point2i loc;
        loc.x = abs(randu<int>()) % (scene.cols-4);
        loc.y = abs(randu<int>()) % (scene.rows-3);
        sig_.Append(NAME_LOC, static_cast<Mat1f>(elm::Point2Mat(loc)));

        to_->Activate(sig_);
        to_->Response(sig_);
        EXPECT_TRUE(sig_.Exists(NAME_TL));
        EXPECT_TRUE(sig_.Exists(NAME_WIN));

        Point2i tl = elm::Mat2Point2i(sig_.MostRecentMat1f(NAME_TL));

        Mat1f window = sig_.MostRecentMat1f(NAME_WIN);
        EXPECT_MAT_DIMS_EQ(window, Size2i(4, 3));

        EXPECT_MAT_EQ(scene(Rect(tl, window.size())), window);
    }
}

TEST_F(AttentionWindowTest, Window_scene_too_small)
{
    sig_.Append(NAME_SCENE, Mat1f(2, 2));
    EXPECT_THROW(to_->Activate(sig_), ExceptionBadDims);
}

TEST_F(AttentionWindowTest, Loc_NoRectify)
{
    const Mat1f scene = sig_.MostRecentMat1f(NAME_SCENE);
    const int WIN_ROWS=3;
    const int WIN_COLS=4;
    const Size2i WIN_SIZE(WIN_COLS, WIN_ROWS);

    for(int x=0; x<scene.cols-WIN_COLS; x++) {

        for(int y=0; y<scene.rows-WIN_ROWS; y++) {

            Point2i tl_expected(x, y);
            Point2i loc = TL2Centre(tl_expected, WIN_SIZE);
            sig_.Append(NAME_LOC, static_cast<Mat1f>(elm::Point2Mat(loc)));

            to_->Activate(sig_);
            to_->Response(sig_);

            EXPECT_TRUE(sig_.Exists(NAME_TL));
            Point2i tl_actual = elm::Mat2Point2i(sig_.MostRecentMat1f(NAME_TL));

            EXPECT_EQ(tl_expected, tl_actual);
        }
    }
}

TEST_F(AttentionWindowTest, Loc_Borders)
{
    const Mat1f scene = sig_.MostRecentMat1f(NAME_SCENE);
    const int WIN_ROWS=3;
    const int WIN_COLS=4;
    const Size2i WIN_SIZE(WIN_COLS, WIN_ROWS);

    // top
    for(int x=0; x<scene.cols-WIN_COLS; x++) {

        int y=0;
        Point2i tl_expected(x, y);
        Point2i loc = TL2Centre(tl_expected, WIN_SIZE);
        loc.y = y;

        sig_.Append(NAME_LOC, static_cast<Mat1f>(elm::Point2Mat(loc)));

        to_->Activate(sig_);
        to_->Response(sig_);

        EXPECT_TRUE(sig_.Exists(NAME_TL));
        EXPECT_EQ(tl_expected, elm::Mat2Point2i(sig_.MostRecentMat1f(NAME_TL)));
    }

    // left
    for(int y=0; y<scene.rows-WIN_ROWS; y++) {

        int x=0;
        Point2i tl_expected(x, y);
        Point2i loc = TL2Centre(tl_expected, WIN_SIZE);
        loc.x = x;

        sig_.Append(NAME_LOC, static_cast<Mat1f>(elm::Point2Mat(loc)));

        to_->Activate(sig_);
        to_->Response(sig_);

        EXPECT_TRUE(sig_.Exists(NAME_TL));
        EXPECT_EQ(tl_expected, elm::Mat2Point2i(sig_.MostRecentMat1f(NAME_TL)));
    }

    // right
    for(int y=0; y<scene.rows-WIN_ROWS; y++) {

        int x=scene.cols-1;
        Point2i tl_expected(x, y);
        Point2i loc = TL2Centre(tl_expected, WIN_SIZE);
        loc.x = x;
        tl_expected.x = scene.cols-WIN_COLS;

        sig_.Append(NAME_LOC, static_cast<Mat1f>(elm::Point2Mat(loc)));

        to_->Activate(sig_);
        to_->Response(sig_);

        EXPECT_TRUE(sig_.Exists(NAME_TL));
        EXPECT_EQ(tl_expected, elm::Mat2Point2i(sig_.MostRecentMat1f(NAME_TL)));
    }

    // bottom
    for(int x=0; x<scene.cols-WIN_COLS; x++) {

        int y=scene.rows-1;
        Point2i tl_expected(x, y);
        Point2i loc = TL2Centre(tl_expected, WIN_SIZE);
        loc.y = y;
        tl_expected.y = scene.rows-WIN_ROWS;

        sig_.Append(NAME_LOC, static_cast<Mat1f>(elm::Point2Mat(loc)));

        to_->Activate(sig_);
        to_->Response(sig_);

        EXPECT_TRUE(sig_.Exists(NAME_TL));
        EXPECT_EQ(tl_expected, elm::Mat2Point2i(sig_.MostRecentMat1f(NAME_TL)));
    }
}

TEST_F(AttentionWindowTest, Clear)
{
    to_->Clear();
    to_->Response(sig_);

    EXPECT_TRUE(sig_.Exists(NAME_WIN));
    EXPECT_MAT_DIMS_EQ(sig_.MostRecentMat1f(NAME_WIN), Size2i(4, 3));

    EXPECT_TRUE(sig_.Exists(NAME_TL));
    EXPECT_EQ(Point2i(-1, -1), elm::Mat2Point2i(sig_.MostRecentMat1f(NAME_TL)));
}

} // anonnymous namespace around test fixtures



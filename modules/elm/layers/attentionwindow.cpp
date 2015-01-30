/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, Elm Library Project
// 3-clause BSD License
//
//M*/
#include "elm/layers/attentionwindow.h"

#include "elm/core/exception.h"
#include "elm/core/cv/mat_utils.h"
#include "elm/core/signal.h"
#include "elm/ts/layerattr_.h"

using namespace cv;
using namespace std;
using namespace elm;

const string AttentionWindow::PARAM_WIN_COLS = "cols";
const string AttentionWindow::PARAM_WIN_ROWS = "rows";

const string AttentionWindow::KEY_INPUT_LOC      = "loc";
const string AttentionWindow::KEY_INPUT_SCENE    = "scene";
const string AttentionWindow::KEY_OUTPUT_WIN     = "window";
const string AttentionWindow::KEY_OUTPUT_OPT_TL = "loc_new";

/** @todo why does define guard lead to undefined reference error?
 */
//#ifdef __WITH_GTEST
#include <boost/assign/list_of.hpp>
template <>
elm::MapIONames LayerAttr_<AttentionWindow>::io_pairs = boost::assign::map_list_of
        ELM_ADD_INPUT_PAIR(AttentionWindow::KEY_INPUT_LOC)
        ELM_ADD_INPUT_PAIR(AttentionWindow::KEY_INPUT_SCENE)
        ELM_ADD_OUTPUT_PAIR(AttentionWindow::KEY_OUTPUT_WIN)
        ;
//#endif

AttentionWindow::~AttentionWindow()
{
}

AttentionWindow::AttentionWindow()
    : base_Layer(),
      window_(Mat1f())
{
    Clear();
}

AttentionWindow::AttentionWindow(const LayerConfig &cfg)
    : base_Layer(cfg)
{
    Reset(cfg);
}

void AttentionWindow::Clear()
{
    window_ = Mat1f(window_.size());
    tl_.x = tl_.y = -1;
}

void AttentionWindow::Reset(const LayerConfig &config)
{
    PTree p = config.Params();
    int rows = p.get<int>(PARAM_WIN_ROWS);
    int cols = p.get<int>(PARAM_WIN_COLS);

    window_ = Mat1f(rows, cols);
}

void AttentionWindow::Reconfigure(const LayerConfig &config)
{
    Reset(config);
}

void AttentionWindow::IONames(const LayerIONames &config)
{
    name_in_loc_ = config.Input(KEY_INPUT_LOC);
    name_in_scene_ = config.Input(KEY_INPUT_SCENE);
    name_out_win_ = config.Output(KEY_OUTPUT_WIN);

    name_out_tl_ = config.OutputOpt(KEY_OUTPUT_OPT_TL);
}

void AttentionWindow::Activate(const Signal &signal)
{
    Mat1i loc_mat = signal.MostRecentMat(name_in_loc_);
    Mat1f scene = signal.MostRecentMat(name_in_scene_);

    if(scene.rows < window_.rows || scene.cols < window_.cols) {

        stringstream s;
        s << "Cannot extract a " <<
             window_.rows << "x" << window_.cols <<
             " window from a " <<
             scene.rows << "x" << scene.cols <<
             "scene.";
        ELM_THROW_BAD_DIMS(s.str());
    }

    tl_ = elm::Mat2Point2i(loc_mat);

    // rectify location: shift it so that the window falls entirly within the scene
    tl_.x = RectifyCoord(tl_.x, window_.cols, scene.cols);
    tl_.y = RectifyCoord(tl_.y, window_.rows, scene.rows);

    window_ = Mat(scene, Rect2i(tl_, window_.size()));
}

void AttentionWindow::Response(Signal &signal)
{
    signal.Append(name_out_win_, window_);

    // optional outputs
    if(name_out_tl_) {
        signal.Append(name_out_tl_.get(), static_cast<Mat1f>(elm::Point2Mat(tl_)));
    }
}

int AttentionWindow::RectifyCoord(int center, int window_dim, int scene_dim) const
{
    int radius = static_cast<int>(ceil(static_cast<double>(window_dim)/2.));
    int coordinate = center - radius;

    if(coordinate < 0) {

        coordinate = 0;
    }
    else if(coordinate + window_dim - 1 >= scene_dim) {

        coordinate = scene_dim - window_dim;
    }
    return coordinate;
}

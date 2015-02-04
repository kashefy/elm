/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_LAYERS_ATTENTIONWINDOW_H_
#define _ELM_LAYERS_ATTENTIONWINDOW_H_

#include <string>

#include <opencv2/core.hpp>

#include "elm/core/base_Layer.h"
#include "elm/core/layerconfig.h"

namespace elm {
/**
 * @brief layer for sampling patch around attended location (e.g. salient location)
 * @todo Figure out where inhibition of return goes.
 */
class AttentionWindow : public base_Layer
{
public:
    // params
    static const std::string PARAM_WIN_ROWS;    ///< no. of rows in window
    static const std::string PARAM_WIN_COLS;    ///< no. of columns in window

    // I/O Names
    static const std::string KEY_INPUT_LOC;      ///< key to attended location
    static const std::string KEY_INPUT_SCENE;    ///< key to scene image
    static const std::string KEY_OUTPUT_WIN;     ///< key to patch around attended location
    static const std::string KEY_OUTPUT_OPT_TL;  ///< key to top-left corner of attended patch in scene space (optional output)

    virtual ~AttentionWindow();

    AttentionWindow();

    AttentionWindow(const LayerConfig &cfg);

    virtual void Clear();

    virtual void Reset(const LayerConfig &config);

    virtual void Reconfigure(const LayerConfig &config);

    virtual void InputNames(const LayerIONames &config);

    virtual void OutputNames(const LayerIONames &config);

    virtual void Activate(const Signal &signal);

    virtual void Response(Signal &signal);

protected:
    /**
     * @brief Rectify coordinate for window to overlap with scene along that axis
     * In case the window already lies within the scene boundaries, this yields
     * the window's top-left corner coordinate instead of its center
     *
     * @param center of desired window location
     * @param window dimension along that axis
     * @param scene dimension along that axis
     * @return top-left corner coordinate
     */
    virtual int RectifyCoord(int center, int window_dim, int scene_dim) const;

    std::string name_in_loc_;   ///< name of attended location in signal object
    std::string name_in_scene_; ///< name of stimulus scene image
    std::string name_out_win_;  ///< desintation name of patch around location in signal object
    OptS        name_out_tl_;  ///< optional desitnation to recitfied location

    cv::Mat1f   window_;        ///< window of attention
    cv::Point2i tl_;     ///< window's top-left corner
};

} // namespace elm

#endif // _ELM_LAYERS_ATTENTIONWINDOW_H_

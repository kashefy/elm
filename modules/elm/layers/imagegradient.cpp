/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/layers/imagegradient.h"

#include <vector>

#include "elm/core/debug_utils.h"
#include "elm/core/exception.h"
#include "elm/core/signal.h"
#include "elm/ts/layerattr_.h"

using namespace cv;
using namespace elm;

/** @todo why does define guard lead to undefined reference error?
 */
//#ifdef __WITH_GTEST
#include <boost/assign/list_of.hpp>
template <>
elm::MapIONames LayerAttr_<ImageGradient>::io_pairs = boost::assign::map_list_of
        ELM_ADD_INPUT_PAIR(elm::detail::BASE_SINGLE_INPUT_FEATURE_LAYER__KEY_INPUT_STIMULUS)
        ELM_ADD_OUTPUT_PAIR(elm::detail::BASE_MATOUTPUT_LAYER__KEY_OUTPUT_RESPONSE)
        ;
//#endif

ImageGradient::ImageGradient()
    : base_FeatureTransformationLayer() {

    Clear();
}

void ImageGradient::Clear() {

    m_ = Mat1f();
}

void ImageGradient::Reconfigure(const LayerConfig &config) {

    Clear();
}

void ImageGradient::Activate(const Signal &signal) {

    Mat img = signal.MostRecentMat1f(name_input_);

    ELM_THROW_BAD_DIMS_IF(img.rows < 2, "No. of rows must be >= 2 for computing vertical gradient component");
    ELM_THROW_BAD_DIMS_IF(img.cols < 2, "No. of columns must be >= 2 for computing horizontal gradient component");

    Mat gx, gy; // horizontal and vertical gradient components

    gx = img.colRange(1, img.cols) - img.colRange(0, img.cols-1);
    hconcat(gx, Mat(gx.rows, 1, gx.type(), cv::Scalar(0)), gx);

    gy = img.rowRange(1, img.rows) - img.rowRange(0, img.rows-1);
    gy.push_back(Mat(1, gy.cols, gy.type(), cv::Scalar(0))); // push_back seems to be faster than vconcat

    std::vector<Mat1f> v = {gx, gy};

    Mat tmp;
    cv::merge(v, tmp);

    m_ = static_cast<Mat1f>(tmp);
}

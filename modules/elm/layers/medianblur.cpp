/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/layers/medianblur.h"

#include <opencv2/imgproc/imgproc.hpp>

#include "elm/core/exception.h"
#include "elm/core/layerconfig.h"
#include "elm/core/signal.h"
#include "elm/ts/layerattr_.h"

using namespace cv;
using namespace elm;

/** @todo why does define guard lead to undefined reference error?
 */
//#ifdef __WITH_GTEST
#include <boost/assign/list_of.hpp>
template <>
elm::MapIONames LayerAttr_<MedianBlur>::io_pairs = boost::assign::map_list_of
        ELM_ADD_INPUT_PAIR(detail::BASE_SINGLE_INPUT_FEATURE_LAYER__KEY_INPUT_STIMULUS)
        ELM_ADD_OUTPUT_PAIR(detail::BASE_MATOUTPUT_LAYER__KEY_OUTPUT_RESPONSE)
        ;
//#endif

MedianBlur::MedianBlur()
    : base_SmoothLayer()
{
    Clear();
}

MedianBlur::MedianBlur(const LayerConfig &config)
    : base_SmoothLayer(config)
{
    Reset(config);
    IONames(config);
}

void MedianBlur::Reconfigure(const LayerConfig &config)
{
    base_SmoothLayer::Reconfigure(config);

    if(ksize_ <= 1 || ksize_ % 2 == 0) {

        std::stringstream s;
        s << "Aperture size for median filer must be > 1 and odd." <<
             " Got " << ksize_;
        ELM_THROW_VALUE_ERROR(s.str());
    }
}

void MedianBlur::Activate(const Signal &signal)
{
    Mat src, dst;

    /* from OpneCV's docs:
     * when ksize is 3 or 5,
     * the image depth should be CV_8U, CV_16U, or CV_32F,
     * for larger aperture sizes, it can only be CV_8U
     **/
    if(ksize_ > 5) {

        signal.MostRecentMat1f(name_input_).convertTo(src, CV_8U);
    }
    else {

        src = signal.MostRecentMat1f(name_input_);

        Mat1b nan = elm::isnan(src);
        if(cv::countNonZero(nan) > 0) {

            src = src.clone();
            src.setTo(1e7, nan);
        }
    }

    medianBlur(src, dst, ksize_);
    m_ = dst;
}

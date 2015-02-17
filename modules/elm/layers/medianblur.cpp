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

/** Define parameters, defaults and I/O keys
  */
// paramters
const std::string MedianBlur::PARAM_APERTURE_SIZE = "aperture_size";

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
    : base_FeatureTransformationLayer()
{
    Clear();
}

MedianBlur::MedianBlur(const LayerConfig config)
    : base_FeatureTransformationLayer(config)
{
    Reset(config);
    IONames(config);
}

void MedianBlur::Clear()
{
    m_ = Mat1f();
}

void MedianBlur::Reset(const LayerConfig &config)
{
    Clear();
    Reconfigure(config);
}

void MedianBlur::Reconfigure(const LayerConfig &config)
{
    PTree p = config.Params();
    ksize_ = p.get<int>(PARAM_APERTURE_SIZE);

    if(ksize_ <= 1 || ksize_ % 2 == 0) {

        std::stringstream s;
        s << "Aperture size for median filer must be > 1 and odd." <<
             " Got " << ksize_;
        ELM_THROW_VALUE_ERROR(s.str());
    }
}

void MedianBlur::Activate(const Signal &signal)
{
    Mat in;
    signal.MostRecentMat1f(name_input_).convertTo(in, CV_8UC1);
    Mat tmp(in.size(), CV_8UC1);
    medianBlur(in, tmp, ksize_); // OpenCV: it can only be CV_8U...
    m_ = tmp;
}

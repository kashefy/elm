#include "layers/saliencyitti.h"

#include <opencv2/imgproc.hpp>

#include "core/defs.h"
#include "core/layerconfig.h"
#include "core/mat_utils.h"
#include "core/percentile.h"
#include "core/signal.h"
#include "encoding/orientation.h"

using namespace std;
using namespace cv;

using namespace sem;

/** Define paramter names and IO keys
  */
const std::string SaliencyItti::KEY_INPUT_SCENE         = "scene";          ///< key to visual scene stimulus
const std::string SaliencyItti::KEY_OUTPUT_SALIENCY     = "saliency";       ///< key to saliency measure
const std::string SaliencyItti::KEY_OUTPUT_SALIENT_LOC  = "salient_loc";    ///< key to most recently attended location

const float SaliencyItti::DEFAULT_SIGMA         = 3.f;
const float SaliencyItti::DEFAULT_LAMBDA        = 10.f;
const float SaliencyItti::DEFAULT_GAMMA         = 0.02f;
const float SaliencyItti::DEFAULT_PS            = 0.f;    ///< [radians]
const float SaliencyItti::DEFAULT_ORIENT_RESPONSE_PERCENTILE = 0.7f;

SaliencyItti::~SaliencyItti()
{
}

SaliencyItti::SaliencyItti()
    : base_Layer()
{
    Clear();
}

SaliencyItti::SaliencyItti(const LayerConfig &config)
    : base_Layer(config)
{
    Reset(config);
}

void SaliencyItti::Clear()
{
    saliency_ = Mat1f();
    percentile_orientation_response_ = DEFAULT_ORIENT_RESPONSE_PERCENTILE;
}

void SaliencyItti::Reset(const LayerConfig &config)
{
    // orientation conspicuity
    theta_range_ = ARange_<float>(0.f, CV_PI, 90.*CV_PI/180.);
    VecF theta = Mat_ToVec_<float>(theta_range_);

    gabors_.reset(new GaborFilterBank());
    SEM_DYN_CAST(GaborFilterBank, gabors_)->Reset(DEFAULT_RADIUS,
                                                  DEFAULT_SIGMA,
                                                  theta,
                                                  DEFAULT_LAMBDA,
                                                  DEFAULT_GAMMA,
                                                  DEFAULT_PS);

    // intensity contrast
    intensity_constrast_.Init(DEFAULT_RADIUS, 1.f);

    percentile_orientation_response_ = DEFAULT_ORIENT_RESPONSE_PERCENTILE;
}

void SaliencyItti::Reconfigure(const LayerConfig &config)
{
    Reset(config);
}

void SaliencyItti::IONames(const LayerIONames &config)
{
    name_scene_     = config.Input(KEY_INPUT_SCENE);
    name_saliency_  = config.Output(KEY_OUTPUT_SALIENCY);
    name_salient_loc_ = config.Output(KEY_OUTPUT_SALIENT_LOC);
}

void SaliencyItti::Activate(const Signal &signal)
{
    Mat1f stimulus = signal.MostRecent(name_scene_);
    pop_code_orient_.State(stimulus, gabors_);

    Mat orientation_spikes = pop_code_orient_.PopCode();
    Mat1i orientation_index(stimulus.size());

    Mat1f orientation_response_flat = Reshape(gabors_->Response()).reshape(1, 1);
    float low_orientation_response_percentile = Percentile()
            .CalcPercentile(orientation_response_flat,
                            percentile_orientation_response_);
    Mat1b mask_low_orientation_response(stimulus.size());

    // assing orientation to each stimulus pixel
    size_t j_step = gabors_->size();
    double min_val;
    int max_idx[2];
    for(size_t i=0, j=0; i<stimulus.total(); i++, j+=j_step) {

        minMaxIdx(orientation_spikes.colRange(j, j+j_step), &min_val, 0, 0, max_idx);
        orientation_index(i) = max_idx[1];

        mask_low_orientation_response(i) = orientation_response_flat(j+max_idx[1]) <= low_orientation_response_percentile;
    }

    // shift indices to be zero-centered.
    // subtraction should work with even as well as odd sizes.
    orientation_index -= gabors_->size()/2;

    Mat1f orientation_conspicuity, tmp;
    NeighMeanVar(orientation_index, 3, tmp, orientation_conspicuity);
    normalize(orientation_conspicuity, orientation_conspicuity, 0.f, 255.f, NORM_MINMAX, -1, noArray());

    // apply feedback on orientation measurements
    // clip away low response elements
    orientation_conspicuity.setTo(0.f, mask_low_orientation_response);

    //TODO: weigh by entropy to clip away uniform response regions, use as feedback signal

    intensity_constrast_.Compute(stimulus);
    Mat intensity_constrast_norm = intensity_constrast_.Response();
    normalize(intensity_constrast_norm, intensity_constrast_norm, 0.f, 255.f, NORM_MINMAX, -1, noArray());

    add(intensity_constrast_norm, orientation_conspicuity, saliency_);

    // generate 2d distribution from saliency map for later sampling of salient locations
    saliency_sampler_.pdf(saliency_);
}

void SaliencyItti::Response(Signal &signal)
{
    signal.Append(name_saliency_, saliency_);

    Point2i loc = saliency_sampler_.Sample();
    Mat1i loc_mat = Point2Mat(loc);

    signal.Append(name_salient_loc_, loc_mat);
}
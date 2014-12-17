#include "layers/saliencyitti.h"

#include <iostream>
#include <opencv2/imgproc.hpp>

#include "core/signal.h"
#include "core/mat_utils.h"
#include "encoding/orientation.h"

using namespace std;
using namespace cv;

using namespace sem;

/** Define paramter names and IO keys
  */
const std::string SaliencyItti::KEY_INPUT_SCENE         = "scene";          ///< key to visual scene stimulus
const std::string SaliencyItti::KEY_OUTPUT_SALIENCY     = "saliency";       ///< key to saliency measure
const std::string SaliencyItti::KEY_OUTPUT_SALIENT_LOC  = "salient_loc";    ///< key to most recently attended location

SaliencyItti::~SaliencyItti()
{
}

SaliencyItti::SaliencyItti()
{
    Reset();
}

void SaliencyItti::Reset()
{
    saliency_ = Mat1f();
    stimulus_ = Mat1f();

    const int RADIUS = 9;
    const float SIGMA = 3;
    const float _LAMBDA = 10;   //CV_PI;
    const float GAMMA = 0.02;   //10;
    const float PS = 0;         //CV_PI*0.5;

    theta_range_ = ARange_<float>(0.f, CV_PI, 90.*CV_PI/180.);

    // Mat1f to VecF
    const float* p = theta_range_.ptr<float>(0);
    VecF theta(p, p+theta_range_.cols);

    kernels_orient_ = GaborFilterBank(RADIUS, SIGMA, theta, _LAMBDA, GAMMA, PS);

    intensity_constrast_.Init(RADIUS, 1.f);
}

void SaliencyItti::Reconfigure(const LayerConfig &config)
{
    name_scene_ = config.Input(KEY_INPUT_SCENE);
    name_saliency_ = config.Output(KEY_OUTPUT_SALIENCY);
    name_salient_loc_ = config.Output(KEY_OUTPUT_SALIENT_LOC);
}

void SaliencyItti::Stimulus(const Signal &signal)
{
    stimulus_ = signal.MostRecent(name_scene_);
    pop_code_orient_.State(stimulus_, kernels_orient_);
}

void SaliencyItti::Apply()
{
    Mat orientation_spikes = pop_code_orient_.PopCode();
    Mat1i orientation_index(stimulus_.size());

    // assing orientation to each stimulus pixel
    size_t j_step = kernels_orient_.size();
    double min_val;
    int max_idx[2];
    for(size_t i=0, j=0; i<stimulus_.total(); i++, j+=j_step) {

        minMaxIdx(orientation_spikes.colRange(j, j+j_step), &min_val, 0, 0, max_idx);
        cout<<i<<" "<<orientation_spikes.colRange(j, j+j_step)<<" "<<max_idx[1]<<endl;
        orientation_index(i) = max_idx[1];
    }

    // shift indices to be zero-centered.
    // subtraction should work with even as well as odd sizes.
    orientation_index -= kernels_orient_.size()/2;

    Mat1f orientation_conspicuity, tmp;
    NeighMeanVar(orientation_index, 3, tmp, orientation_conspicuity);
    normalize(orientation_conspicuity, orientation_conspicuity, 0.f, 255.f, NORM_MINMAX, -1, noArray());

    intensity_constrast_.Compute(stimulus_);
    Mat intensity_constrast_norm = intensity_constrast_.Response();
    normalize(intensity_constrast_norm, intensity_constrast_norm, 0.f, 255.f, NORM_MINMAX, -1, noArray());

    add(1.f*intensity_constrast_norm, 0*orientation_conspicuity, saliency_, noArray());

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

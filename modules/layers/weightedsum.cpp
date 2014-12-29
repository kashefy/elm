#include "layers/weightedsum.h"

#include "core/exception.h"
#include "core/layerconfig.h"
#include "core/signal.h"

using namespace std;
using namespace cv;

/** Define parameters and I/O keys
  */
const std::string WeightedSum::PARAM_A              = "a";
const std::string WeightedSum::PARAM_B              = "b";
const std::string WeightedSum::KEY_INPUT_STIMULUS   = "in";
const std::string WeightedSum::KEY_OUTPUT_RESPONSE  =  "out";

void WeightedSum::Clear()
{
    stimulus_ = Mat1f();
    response_ = Mat1f();
}

void WeightedSum::Reconfigure(const LayerConfig &config)
{
    // params
    PTree params = config.Params();
    a_ = params.get<float>(PARAM_A);
    b_ = params.get<float>(PARAM_B);
}

void WeightedSum::IONames(const LayerIONames &config)
{
    name_stimulus_ = config.Input(KEY_INPUT_STIMULUS);
    name_response_ = config.Output(KEY_OUTPUT_RESPONSE);
}

void WeightedSum::Stimulus(const Signal &signal)
{
    stimulus_ = signal[name_stimulus_][0];
    if(stimulus_.cols > 2) {

        SEM_THROW_BAD_DIMS("Cannot handle stimulus with > 2 columns.");
    }
}

void WeightedSum::Apply()
{
    response_ = Mat1f(stimulus_.rows, 1);
    for(int r=0; r<stimulus_.rows; r++) {

        float tmp = a_ * stimulus_(r, 0);
        if(stimulus_.cols > 1) {

            tmp += b_ * stimulus_(r, 1);
        }
        response_(r) = tmp;
    }
}

void WeightedSum::Response(Signal &signal)
{
    signal.Append(name_response_, response_);
}

WeightedSum::WeightedSum()
    : base_Layer()
{
    Clear();
}

WeightedSum::WeightedSum(const LayerConfig& config)
    : base_Layer(config)
{
    Clear();
    Reconfigure(config);
    IONames(config);
}

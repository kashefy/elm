#include "layers/icp.h"

#ifdef __WITH_PCL   // the layer is otherwise implemented as unsupported

#include "core/exception.h"
#include "core/layerconfig.h"
#include "core/mat_utils.h"
#include "core/signal.h"

using namespace std;
using namespace cv;
using namespace sem;

const string ICP::KEY_INPUT_POINT_CLOUD_SRC     = "source";
const string ICP::KEY_INPUT_POINT_CLOUD_TARGET  = "target";
const string ICP::KEY_OUTPUT_CONVERGENCE        = "convergence";
const string ICP::KEY_OUTPUT_SCORE              = "score";
const string ICP::KEY_OUTPUT_TRANSFORMATION     = "transformation";

ICP::ICP()
{
}

ICP::ICP(const LayerConfig &cfg)
{
}

void ICP::Clear()
{

}

void ICP::Reconfigure(const LayerConfig &cfg)
{

}

void ICP::Reset()
{

}

void ICP::Reset(const LayerConfig &cfg)
{

}

void ICP::IONames(const LayerIONames &io)
{
    // input names
    name_src_cloud_     = io.Input(KEY_INPUT_POINT_CLOUD_SRC);
    name_target_cloud_  = io.Input(KEY_INPUT_POINT_CLOUD_TARGET);

    // output names
    name_convergence_   = io.Output(KEY_OUTPUT_CONVERGENCE);
    name_score_         = io.Output(KEY_OUTPUT_SCORE);
    name_transf_        = io.Output(KEY_OUTPUT_TRANSFORMATION);
}

void ICP::Activate(const Signal &signal)
{

}

void ICP::Response(Signal &signal)
{

}

#endif // __WITH_PCL

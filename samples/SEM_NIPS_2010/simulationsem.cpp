#include "simulationsem.h"

#include <iostream>

#include <boost/filesystem.hpp>

#include <opencv2/highgui.hpp>

#include "elm/core/signal.h"
#include "elm/core/cv/mat_utils.h"
#include "elm/encoding/populationcode.h"
#include "elm/io/readmnist.h"
#include "elm/layers/layerfactory.h"
#include "elm/layers/layer_y.h"
#include "elm/layers/layer_z.h"

using namespace cv;
using namespace std;
namespace bfs=boost::filesystem;

const string SimulationSEM::NAME_STIMULUS  = "stimulus";
const string SimulationSEM::NAME_POP_CODE  = "pc";
const string SimulationSEM::NAME_SPIKES_Y  = "y";
const string SimulationSEM::NAME_SPIKES_Z  = "z";
const string SimulationSEM::NAME_WEIGHTS   = "w";

SimulationSEM::SimulationSEM()
    : nb_learners_(40)
{
    pop_code_ = InitPopulationCode();
    y_ = InitLayerY();
}

void SimulationSEM::Learn()
{
    ReadMNISTImages r;
    //bfs::path p("C:\\Users\\woodstock\\dev\\data\\MNIST\\train-images.idx3-ubyte");
    //bfs::path p("C:\\Users\\woodstock\\dev\\data\\MNIST\\t10k-images.idx3-ubyte");
    //bfs::path p("/media/206CDC456CDC177E/Users/woodstock/dev/data/MNIST/train-images.idx3-ubyte");
    bfs::path p("/media/206CDC456CDC177E/Users/woodstock/dev/data/MNIST/t10k-images.idx3-ubyte");
    r.ReadHeader(p.string().c_str());

    Signal sig;

    while(!r.IS_EOF()) {

        sig.Clear();

        Mat1f img = r.Next();

        sig.Append(NAME_STIMULUS, img);

        pop_code_->Activate(sig);
        pop_code_->Response(sig);

        const int T=20;
        for(int t=0; t<T; t++) {

            y_->Activate(sig);
            y_->Response(sig);

            if(!z_) {

                z_ = InitLearners(static_cast<int>(sig.MostRecentMat(NAME_SPIKES_Y).total()), 10);
            }

            z_->Activate(sig);
            static_pointer_cast<base_LearningLayer>(z_)->Learn();
        }

        z_->Clear(); // clear before moving on to the next stimulus
    }
}

void SimulationSEM::Test()
{
    ReadMNISTImages r;
    //bfs::path p("C:\\Users\\woodstock\\dev\\data\\MNIST\\t10k-images.idx3-ubyte");
    bfs::path p("/media/206CDC456CDC177E/Users/woodstock/dev/data/MNIST/t10k-images.idx3-ubyte");
    r.ReadHeader(p.string().c_str());

    Signal sig;

    while(!r.IS_EOF()) {

        sig.Clear();

        Mat1f img = r.Next();
        //cv::imshow("i", img);
        sig.Append(NAME_STIMULUS, img);

        pop_code_->Activate(sig);
        pop_code_->Response(sig);

        y_->Activate(sig);
        y_->Response(sig);

        z_->Activate(sig);
    }
}

void SimulationSEM::Eval()
{
    Signal signal;
    z_->Response(signal);
    SimulationSEM::VisualizeOnOffWeights(signal.MostRecentMat(NAME_WEIGHTS));
}

shared_ptr<base_Layer> SimulationSEM::InitPopulationCode() const
{
    LayerConfig cfg;
    cfg.Input(MutexPopulationCode::KEY_INPUT_STIMULUS, NAME_STIMULUS);
    cfg.Output(MutexPopulationCode::KEY_OUTPUT_POP_CODE, NAME_POP_CODE);

    return LayerFactory::CreateShared("MutexPopulationCode", cfg, cfg);
}

shared_ptr<base_Layer> SimulationSEM::InitLayerY() const
{
    PTree params;
    params.put(LayerY::PARAM_FREQ, 1000.f);
    params.put(LayerY::PARAM_DELTA_T_MSEC, 1.f);

    LayerConfig cfg;
    cfg.Params(params);

    LayerIONames io;
    io.Input(LayerY::KEY_INPUT_STIMULUS, NAME_POP_CODE);
    io.Output(LayerY::KEY_OUTPUT_SPIKES, NAME_SPIKES_Y);

    return LayerFactory::CreateShared("LayerY", cfg, io);
}

shared_ptr<base_Layer> SimulationSEM::InitLearners(int nb_features, int history_length) const
{
    PTree params;
    params.put(LayerZ::PARAM_NB_AFFERENTS, nb_features);
    params.put(LayerZ::PARAM_LEN_HISTORY, history_length);
    params.put(LayerZ::PARAM_NB_OUTPUT_NODES, nb_learners_);
    LayerConfig cfg;
    cfg.Params(params);

    cfg.Input(LayerZ::KEY_INPUT_SPIKES, NAME_SPIKES_Y);
    cfg.Output(LayerZ::KEY_OUTPUT_SPIKES, NAME_SPIKES_Z);
    cfg.Output(LayerZ::KEY_OUTPUT_WEIGHTS, NAME_WEIGHTS);

    return LayerFactory::CreateShared("LayerZ", cfg, cfg);
}

void SimulationSEM::VisualizeOnOffWeights(const Mat1f &weights)
{
    // these weights are interlaced in wether they represent on or off activity
    // we'll reshape the matrix for easier visualization
    Mat1f weights_exp;
    exp(weights, weights_exp); // weights are initially in log scale

    for(size_t i=0; i<nb_learners_; i++) {

        Mat1f w = weights_exp.row(i);
        w = w.reshape(1, static_cast<int>(w.total()/2.));

        Mat1f w_on = w.col(0);
        Mat1f w_off = w.col(1);

        //cout<<w_on.t()<<endl;
        //cout<<w_off.t()<<endl;

        imshow("on", elm::ConvertTo8U(w_on).reshape(1, 28));
        imshow("off", elm::ConvertTo8U(w_off).reshape(1, 28));
        waitKey();
    }
}

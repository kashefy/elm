#include <iostream>

#include <boost/filesystem.hpp>

#include <opencv2/highgui.hpp>

#include "core/core.h"
#include "core/signal.h"
#include "core/mat_utils.h"
#include "encoding/populationcode.h"
#include "io/readmnist.h"
#include "layers/layerfactory.h"
#include "layers/layer_y.h"
#include "layers/layer_z.h"
#include "neuron/wtapoisson.h"
#include "neuron/neuron.h"
#include "neuron/zneuron.h"

using namespace cv;
using namespace std;
namespace bfs=boost::filesystem;

const string NAME_STIMULUS  = "stimulus";
const string NAME_POP_CODE  = "pc";
const string NAME_SPIKES_Y  = "y";
const string NAME_SPIKES_Z  = "z";
const string NAME_WEIGHTS   = "w";

class Simulation
{
public:
    Simulation()
        : nb_learners_(40)
    {
        pop_code_ = InitPopulationCode();
    }

    void Learn()
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

            Mat img = r.Next();

            sig.Append(NAME_STIMULUS, img);

            pop_code_->Stimulus(sig);
            pop_code_->Apply();
            pop_code_->Response(sig);

            Mat1f pc = sig.MostRecent(NAME_POP_CODE);

            YNeuron neuron_y;
            neuron_y.init(1.f, 1000.f);

            const int T=20;
            for(int t=0; t<T; t++) {

                Mat1f spikes_y(1, pc.total());

                for(size_t i=0; i<pc.total(); i++) {

                    spikes_y(i) = neuron_y.State(pc(i));
                }

                if(!z_) {

                    z_ = InitLearners(spikes_y.cols, 10);
                }

                sig.Append(NAME_SPIKES_Y, spikes_y);

                z_->Stimulus(sig);
                z_->Apply();
                static_pointer_cast<base_LearningLayer>(z_)->Learn();
            }

            z_->Clear();
        }
    }

    void Test()
    {
        ReadMNISTImages r;
        //bfs::path p("C:\\Users\\woodstock\\dev\\data\\MNIST\\t10k-images.idx3-ubyte");
        bfs::path p("/media/206CDC456CDC177E/Users/woodstock/dev/data/MNIST/t10k-images.idx3-ubyte");
        r.ReadHeader(p.string().c_str());

        Signal sig;

        while(!r.IS_EOF()) {

            sig.Clear();

            Mat img = r.Next();
            //cv::imshow("i", img);
            sig.Append(NAME_STIMULUS, img);

            pop_code_->Stimulus(sig);
            pop_code_->Apply();
            pop_code_->Response(sig);

            Mat1f pc = sig.MostRecent(NAME_POP_CODE);

            YNeuron neuron_y;
            neuron_y.init(1.f, 1000.f);

            Mat1f spikes_y(1, pc.total());

            for(size_t i=0; i<pc.total(); i++) {

                spikes_y(i) = neuron_y.State(pc(i));
            }

            sig.Append(NAME_SPIKES_Y, spikes_y);

            z_->Stimulus(sig);
            z_->Apply();
        }
    }

    void Eval()
    {
        Signal signal;
        z_->Response(signal);

        Mat1f weights = signal.MostRecent(NAME_WEIGHTS);

        for(size_t i=0; i<nb_learners_; i++) {

            Mat1f w = weights.row(i);
            exp(w, w);

            Mat1f w_on(w.rows, w.cols/2);
            Mat1f w_off(w.rows, w.cols/2);
            for(size_t i=0, j=0; i<w.total(); i+=2, j++) {

                w_on(j) = w(i);
                w_off(j) = w(i+1);
            }

            cout<<w_on<<endl;
            cout<<w_off<<endl;

            imshow("on", sem::ConvertTo8U(w_on).reshape(1, 28));
            imshow("off", sem::ConvertTo8U(w_off).reshape(1, 28));
            waitKey();
        }
    }

private:

    shared_ptr<base_Layer> InitPopulationCode() const
    {
        LayerConfig cfg;
        cfg.Input(MutexPopulationCode::KEY_INPUT_STIMULUS, NAME_STIMULUS);
        cfg.Output(MutexPopulationCode::KEY_OUTPUT_POP_CODE, NAME_POP_CODE);

        return LayerFactory::CreateLayerPtrShared("MutexPopulationCode", cfg, cfg);
    }

    shared_ptr<base_Layer> InitLayerY() const
    {
        PTree params;
        params.put(LayerY::PARAM_FREQ, 1000.f);
        params.put(LayerY::PARAM_DELTA_T_MSEC, 1.f);

        LayerConfig cfg;
        cfg.Params(params);

        LayerIONames io;
        io.Input(LayerY::KEY_INPUT_STIMULUS, NAME_POP_CODE);
        io.Output(LayerY::KEY_OUTPUT_SPIKES, NAME_SPIKES_Y);

        return LayerFactory::CreateLayerPtrShared("LayerY", cfg, io);
    }

    shared_ptr<base_Layer> InitLearners(int nb_features, int history_length) const
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

        return LayerFactory::CreateLayerPtrShared("LayerZ", cfg, cfg);
    }

    shared_ptr<base_Layer> pop_code_;
    shared_ptr<base_Layer> y_;
    shared_ptr<base_Layer> z_;
    size_t nb_learners_;   ///< no. of learners (e.g. ZNeuron)

};

int main() {

    cout<<sem::GetVersion()<<endl;

    Simulation s;

    cout<<"Learn()"<<endl;

    s.Learn();

    cout<<"Test()"<<endl;

    //s.Test();

    cout<<"Eval()"<<endl;

    s.Eval();

    return 0;
}

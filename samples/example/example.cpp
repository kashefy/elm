#include <iostream>

#include <boost/filesystem.hpp>

#include <opencv2/highgui.hpp>

#include "core/core.h"
#include "core/mat_utils.h"
#include "encoding/populationcode.h"
#include "io/readmnist.h"
#include "neuron/wtapoisson.h"
#include "neuron/neuron.h"
#include "neuron/zneuron.h"

using namespace cv;
using namespace std;
namespace bfs=boost::filesystem;

class Simulation
{
public:
    Simulation()
        : nb_learners_(40)
    {

    }

    void Learn()
    {
        ReadMNISTImages r;
        //bfs::path p("C:\\Users\\woodstock\\dev\\data\\MNIST\\train-images.idx3-ubyte");
        bfs::path p("C:\\Users\\woodstock\\dev\\data\\MNIST\\t10k-images.idx3-ubyte");
        r.ReadHeader(p.string().c_str());

        MutexPopulationCode pop_code;

        WTAPoisson wta_z(1.f, 1000.f);

        bool first_pass = true;

        while(!r.IS_EOF()) {

            Mat img = r.Next();

            pop_code.State(img);
            Mat1f pc = pop_code.PopCode();

            YNeuron neuron_y;
            neuron_y.init(1.f, 1000.f);

            const int T=20;
            for(int t=0; t<T; t++) {

                Mat1f spikes_y(1, pc.total());

                for(size_t i=0; i<pc.total(); i++) {

                    spikes_y(i) = neuron_y.State(pc(i));
                }

                if(first_pass) {

                    InitLearners(spikes_y.cols, 10);
                    first_pass = false;
                }

                Mat1f u(1, layer_z_.size());
                for(size_t i=0; i<layer_z_.size(); i++) {

                    u(i) = layer_z_[i]->Predict(spikes_y).at<float>(0);
                }

                Mat1i spikes_z = wta_z.Compete(layer_z_);
                for(size_t i=0; i<layer_z_.size(); i++) {

                    layer_z_[i]->Learn(spikes_z.col(i));
                }
            }

            for(size_t i=0; i<layer_z_.size(); i++) {

                static_pointer_cast<ZNeuron>(layer_z_[i])->Clear();
            }
        }
    }

    void Test()
    {
        ReadMNISTImages r;
        bfs::path p("C:\\Users\\woodstock\\dev\\data\\MNIST\\t10k-images.idx3-ubyte");
        r.ReadHeader(p.string().c_str());

        MutexPopulationCode pop_code;

        WTAPoisson wta_z(1.f, 1000.f);

        while(!r.IS_EOF()) {

            Mat img = r.Next();
            //cv::imshow("i", img);

            pop_code.State(img);

            Mat1f pc = pop_code.PopCode();

            YNeuron neuron_y;
            neuron_y.init(1.f, 1000.f);

            Mat1f spikes_y(1, pc.total());

            for(size_t i=0; i<pc.total(); i++) {

                spikes_y(i) = neuron_y.State(pc(i));
            }

            Mat1f u(1  , layer_z_.size());
            for(size_t i=0; i<layer_z_.size(); i++) {

                u(i) = layer_z_[i]->Predict(spikes_y).at<float>(0);
            }

            //Mat1f spikes_z = wta_z.Compete(layer_z_);
            //waitKey();
        }
    }

    void Eval()
    {
        for(size_t i=0; i<layer_z_.size(); i++) {

            Mat1f w = static_pointer_cast<ZNeuron>(layer_z_[i])->Weights();

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

    void InitLearners(int nb_features, int history_length)
    {
        for(size_t i=0; i<nb_learners_; i++) {

            shared_ptr<ZNeuron> p(new ZNeuron);
            p->init(nb_features, history_length);
            layer_z_.push_back(p);
        }
    }

    vector<std::shared_ptr<base_Learner> > layer_z_;

    int nb_learners_;   ///< no. of learners (e.g. ZNeuron)

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

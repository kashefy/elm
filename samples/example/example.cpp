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
        : nb_learners_(10)
    {

    }

    void Learn()
    {
        ReadMNISTImages r;
        bfs::path p("C:\\Users\\woodstock\\dev\\data\\MNIST\\train-images.idx3-ubyte");
        r.ReadHeader(p.string().c_str());

        MutexPopulationCode pop_code;

        WTAPoisson wta_z(1.f, 1000.f);

        bool first_pass = true;

        int ri=0;
        while(!r.IS_EOF()) {

            //cout<<"i"<<ri++<<endl;
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

            if(first_pass) {

                for(size_t i=0; i<nb_learners_; i++) {

                    shared_ptr<ZNeuron> p(new ZNeuron);
                    p->init(spikes_y.total(), 10);
                    layer_z_.push_back(p);
                }
            }

            Mat1f u(1, layer_z_.size());
            for(size_t i=0; i<layer_z_.size(); i++) {

                u(i) = layer_z_[i]->Predict(spikes_y).at<float>(0);
            }

            //cout<<"x6"<<layer_z_.size()<<endl;
            Mat1i spikes_z = wta_z.Compete(layer_z_);
            //cout<<spikes_z<<endl;
            for(size_t i=0; i<layer_z_.size(); i++) {

                //cout<<spikes_z.col(i)<<endl;
                layer_z_[i]->Learn(spikes_z.col(i));
            }


            //cout<<"x7"<<endl;
            if(first_pass) { first_pass = false; }
            //waitKey();
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

            Mat1f u(1, layer_z_.size());
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
            Mat1f w_on(w.size());
            Mat1f w_off(w.size());
            for(size_t i=0, j=0; i<w.total(); i+=2, j++) {

                w_on(j) = w(i);
                w_off(j) = w(i+1);
            }

            imshow("on", sem::ConvertTo8U(w_on));
            imshow("off", sem::ConvertTo8U(w_off));
            waitKey();
        }
    }

private:
    vector<std::shared_ptr<base_Learner> > layer_z_;

    int nb_learners_;   ///< no. of learners (e.g. ZNeuron)

};

int main() {

    cout<<sem::GetVersion()<<endl;

    Simulation s;

    cout<<"Learn()"<<endl;

    s.Learn();

    cout<<"Test()"<<endl;

    s.Test();

    cout<<"Eval()"<<endl;

    s.Eval();

    return 0;
}

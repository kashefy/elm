#include <iostream>

#include <boost/filesystem.hpp>

#include <opencv2/highgui.hpp>

#include "core/core.h"
#include "encoding/populationcode.h"
#include "io/readmnist.h"
#include "neuron/wtapoisson.h"
#include "neuron/neuron.h"
#include "neuron/zneuron.h"

using namespace cv;
using namespace std;
namespace bfs=boost::filesystem;

int main() {

    std::cout<<sem::GetVersion();
    ReadMNISTImages r;
    bfs::path p("C:\\Users\\woodstock\\dev\\data\\MNIST\\t10k-images.idx3-ubyte");
    r.ReadHeader(p.string().c_str());

    MutexPopulationCode pop_code;

    vector<std::shared_ptr<base_Learner> > layer_z;
    WTAPoisson wta_z(1.f, 1000.f);

    bool first_pass = true;

    while(!r.IS_EOF()) {

        Mat img = r.Next();
        cv::imshow("i", img);

        pop_code.State(img);

        Mat1f pc = pop_code.PopCode();

        YNeuron neuron_y;
        neuron_y.init(1.f, 1000.f);

        Mat1f spikes_y(1, pc.total());

        for(size_t i; i<pc.total(); i++) {

            spikes_y(i) = neuron_y.State(pc(i));
        }

        if(first_pass) {

            for(size_t i; i<spikes_y.total(); i++) {

                shared_ptr<ZNeuron> p(new ZNeuron);
                p->init(spikes_y.total(), 10);
            }
        }

        Mat1f u(1, layer_z.size());
        for(size_t i; i<layer_z.size(); i++) {

            u(i) = layer_z[i]->Predict(spikes_y).at<float>(0);
        }

        wta_z.Compete(layer_z);

        if(first_pass) { first_pass = false; }
        waitKey();
    }

    return 0;
}

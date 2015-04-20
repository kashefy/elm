#include <iostream>

#include <boost/filesystem.hpp>

#include <opencv2/highgui/highgui.hpp>

#include "elm/core/core.h"
#include "elm/core/cv/mat_utils.h"
#include "elm/core/debug_utils.h"
#include "elm/core/layerconfig.h"
#include "elm/core/inputname.h"
#include "elm/io/readnyudepthv2labeled.h"
#include "elm/layers/layerfactory.h"
#include "elm/layers/mlp.h"

using namespace std;
namespace bfs=boost::filesystem;
using namespace cv;
using namespace elm;

int main() {

    cout<<elm::GetVersion()<<endl;

    bfs::path p("/media/win/Users/woodstock/dev/data/nyu_depth_v2_labeled.mat");

    ReadNYUDepthV2Labeled reader;
    int nb_items = reader.ReadHeader(p.string());

    ELM_COUT_VAR(nb_items);

    for(int i=0; i<10; i++) {

        ELM_COUT_VAR(i);

        cv::Mat bgr, labels, depth;
        reader.Next(bgr, depth, labels);

        cv::imshow("image", bgr);
        cv::imshow("depth", elm::ConvertTo8U(depth));
        cv::imshow("labels", elm::ConvertTo8U(labels));
        cv::waitKey(0);
    }

    return 0;
}

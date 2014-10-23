#include <iostream>

#include <boost/filesystem.hpp>

#include <opencv2/highgui.hpp>

#include "core/core.h"
#include "io/readmnist.h"

using namespace std;
namespace bfs=boost::filesystem;

int main() {

    std::cout<<sem::GetVersion();
    ReadMNISTImages r;
    bfs::path p("C:\\Users\\woodstock\\dev\\data\\MNIST\\t10k-images.idx3-ubyte");
    r.ReadHeader(p.string().c_str());
    while(!r.IS_EOF()) {

        cv::imshow("i", r.Next());
        cv::waitKey();
    }

    return 0;
}

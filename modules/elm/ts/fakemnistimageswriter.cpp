#include "elm/ts/fakemnistimageswriter.h"

#include <opencv2/core/core.hpp>

using namespace std;
namespace bfs=boost::filesystem; // use alias
using cv::Mat;
using namespace elm;

FakeMNISTImagesWriter::FakeMNISTImagesWriter(const bfs::path& p)
    : FakeMNISTLabelsWriter(p)
{
}

void FakeMNISTImagesWriter::SaveHeader(int magic_number)
{
    FakeMNISTLabelsWriter::SaveHeader(magic_number);
    WriteInt(ROWS);
    WriteInt(COLS);
}

void FakeMNISTImagesWriter::SaveItems()
{
    for(int i=0; i<NB_ITEMS; i++) {

        unsigned char c = static_cast<unsigned char>(i % 255);
        cv::Mat next_img = cv::Mat(ROWS, COLS, CV_8UC1, c);
        out_.write(reinterpret_cast<char*>(next_img.data), sizeof(unsigned char)*next_img.total());
    }
}

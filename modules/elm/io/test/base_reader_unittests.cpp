/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/io/base_reader.h"

#include <fstream>

#include <boost/filesystem.hpp>

#include <opencv2/core/core.hpp>

#include "elm/core/layerconfig.h"
#include "elm/core/inputname.h"
#include "elm/core/signal.h"
#include "elm/ts/layer_assertions.h"
#include "elm/ts/mat_assertions.h"

using namespace std;
namespace bfs=boost::filesystem;
using cv::Mat1f;
using namespace elm;

const bfs::path TEST_FILEPATH("tmp.txt");

class DummyReader : public base_Reader
{
public:
    static const std::string KEY_OUTPUT_X;

    void OutputNames(const LayerOutputNames &io) {

        name_out_x_ = io.Output(KEY_OUTPUT_X);
    }

    void Next(Signal &signal) {

        float x;
        in_ >> x;
        signal.Append(name_out_x_, Mat1f(1, 1, x));
    }

    int ReadHeader(const string &path) {

        in_.open(path.c_str());

        float n;
        in_ >> n;
        return n;
    }

protected:
    // members
    std::string name_out_x_;
    ifstream in_;
};
const std::string DummyReader::KEY_OUTPUT_X = "x";

class ReaderTest : public ::testing::Test
{
protected:
    static void SetUpTestCase() {

        ofstream out;
        out.open(TEST_FILEPATH.string().c_str());
        out << 3 << endl;
        out << 11 << endl;
        out << 12 << endl;
        out << 13 << endl;
        out.close();
    }

    virtual void SetUp() {

        LayerConfig cfg;
        PTree p;
        p.put(DummyReader::PARAM_PATH, TEST_FILEPATH);
        cfg.Params(p);

        LayerIONames io;
        io.Output(DummyReader::KEY_OUTPUT_X, "x");

        to_.reset(new DummyReader);
        to_->Reset(cfg);
        to_->IONames(io);
    }

    static void TearDownTestCase() {

        if(bfs::is_regular_file(TEST_FILEPATH)) {

            bfs::remove(TEST_FILEPATH);
        }
        ASSERT_FALSE(bfs::is_regular_file(TEST_FILEPATH)) << "test file was not removed properly.";
    }

    shared_ptr<base_Layer> to_; ///< test object
};

TEST_F(ReaderTest, Constructor) {

}

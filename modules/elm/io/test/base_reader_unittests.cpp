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
#include "elm/ts/container.h"
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

    ~DummyReader() {

        if(in_.is_open()) {

            in_.close();
        }
    }

    void OutputNames(const LayerOutputNames &io) {

        name_out_x_ = io.Output(KEY_OUTPUT_X);
    }

    void Next(Signal &signal) {

        float x;
        in_ >> x;
        signal.Append(name_out_x_, Mat1f(1, 1, x));
    }

    int ReadHeader(const string &path) {

        if(in_.is_open()) {

            in_.close();
        }
        in_.open(path.c_str());

        int n = 0;
        in_ >> n;

        return n;
    }

    DummyReader()
        : base_Reader()
    {}

    DummyReader(const LayerConfig &cfg)
        : base_Reader(cfg)
    {}

protected:
    // members
    std::string name_out_x_;
    ifstream in_;
};
const std::string DummyReader::KEY_OUTPUT_X = "x";

#include <boost/assign/list_of.hpp>
template <>
elm::MapIONames LayerAttr_<DummyReader>::io_pairs = boost::assign::map_list_of
        ELM_ADD_OUTPUT_PAIR(DummyReader::KEY_OUTPUT_X);

ELM_INSTANTIATE_LAYER_TYPED_TEST_CASE_P(DummyReader);

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

        cfg_ = LayerConfig();
        PTree p;
        p.put(DummyReader::PARAM_PATH, TEST_FILEPATH);
        cfg_.Params(p);

        LayerIONames io;
        io.Output(DummyReader::KEY_OUTPUT_X, DummyReader::KEY_OUTPUT_X);

        to_.reset(new DummyReader);
        to_->Reset(cfg_);
        to_->IONames(io);
    }

    static void TearDownTestCase() {

        if(bfs::is_regular_file(TEST_FILEPATH)) {

            bfs::remove(TEST_FILEPATH);
        }
        ASSERT_FALSE(bfs::is_regular_file(TEST_FILEPATH)) << "test file was not removed properly.";
    }

    LayerConfig cfg_;
    shared_ptr<base_Reader> to_; ///< test object
};

TEST_F(ReaderTest, Constructor_overloaded) {

    EXPECT_NO_THROW(to_.reset(new DummyReader(cfg_)));
}

TEST_F(ReaderTest, Nb_Items) {

    EXPECT_EQ(3, to_->Nb_Items());
    EXPECT_EQ(3, to_->Nb_Items()) << "No. of items changing after subsequent call.";
}

TEST_F(ReaderTest, Is_EOF) {

    ASSERT_GT(to_->Nb_Items(), 0);

    int n = to_->Nb_Items();

    ASSERT_FALSE(to_->Is_EOF());

    int i = 0;
    while(!to_->Is_EOF()) {

        Signal s;
        to_->Activate(s);
        to_->Response(s);
        i++;

        EXPECT_EQ(to_->Nb_Items(), n-i);
    }

    ASSERT_TRUE(to_->Is_EOF());

    EXPECT_EQ(to_->Nb_Items(), 0);
}

TEST_F(ReaderTest, Read) {

    ASSERT_GT(to_->Nb_Items(), 0);

    int n = to_->Nb_Items();

    Signal s;
    while(!to_->Is_EOF()) {

        to_->Activate(s);
        to_->Response(s);
    }

    VecMat x = s[DummyReader::KEY_OUTPUT_X];
    EXPECT_SIZE(n, x);

    float value = 11.f;
    for(size_t i=0; i<x.size(); i++, value+=1.f) {

        EXPECT_MAT_EQ(Mat1f(1, 1, value), x[i]);
    }
}

TEST_F(ReaderTest, Reconfigure_bad_path) {

    PTree p = cfg_.Params();
    p.put(DummyReader::PARAM_PATH, "wrong.txt");
    cfg_.Params(p);

    EXPECT_THROW(to_->Reconfigure(cfg_), ExceptionFileIOError);
}

class DummyReaderNoItems : public DummyReader
{
protected:
    int ReadHeader(const string &path) {

        return 0;
    }
};

TEST_F(ReaderTest, Reconfigure_no_items) {

    to_.reset(new DummyReaderNoItems);
    EXPECT_THROW(to_->Reset(cfg_), ExceptionFileIOError);
}

TEST_F(ReaderTest, Reconfigure) {

    ASSERT_GT(to_->Nb_Items(), 0);

    int n = to_->Nb_Items();

    Signal s;
    while(!to_->Is_EOF()) {

        to_->Activate(s);
        to_->Response(s);
    }

    VecMat x1 = s[DummyReader::KEY_OUTPUT_X];
    EXPECT_SIZE(n, x1);

    EXPECT_TRUE(to_->Is_EOF());
    to_->Reset(cfg_);
    EXPECT_FALSE(to_->Is_EOF());

    s.Clear();
    while(!to_->Is_EOF()) {

        to_->Activate(s);
        to_->Response(s);
    }

    VecMat x2 = s[DummyReader::KEY_OUTPUT_X];
    EXPECT_SIZE(x1.size(), x2);

    for(size_t i=0; i<x1.size(); i++) {

        EXPECT_MAT_EQ(x1[i], x2[i]);
        EXPECT_NE(x1[i].data, x2[i].data);
    }
}

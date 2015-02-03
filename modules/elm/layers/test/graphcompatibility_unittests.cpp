/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, Elm Library Project
// 3-clause BSD License
//
//M*/
#include "elm/layers/graphcompatibility.h"

#include "elm/core/signal.h"
#include "elm/core/layerconfig.h"
#include "elm/layers/layerfactory.h"
#include "elm/ts/layer_assertions.h"

using namespace std;
using namespace boost;
using namespace cv;
using namespace elm;

namespace {

ELM_INSTANTIATE_LAYER_TYPED_TEST_CASE_P(GraphCompatibility);

const string NAME_GRAPH_AB = "G_ab";
const string NAME_GRAPH_IJ = "g_ij";
const string NAME_M        = "m";

class GraphCompatibilityTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        LayerConfig cfg;

        LayerIONames io;
        io.Input(GraphCompatibility::KEY_INPUT_GRAPH_AB, NAME_GRAPH_AB);
        io.Input(GraphCompatibility::KEY_INPUT_GRAPH_IJ, NAME_GRAPH_IJ);
        io.Output(GraphCompatibility::KEY_OUTPUT_M, NAME_M);

        to_ = LayerFactory::CreateShared("GraphCompatibility", cfg, io);

        // initialize test graphs
        const int A=4;  ///< no. of nodes in G
        const int I=A;  ///< no. of nodes in g

        // generate random adjacency matrices
        g_ab_ = Mat1f(A, A);
        Mat1i tmp(A, A);

        int s = 1000;
        randu(tmp, 0, s);
        g_ab_ = tmp/static_cast<float>(s);

        // distance from node to itself is 0
        for(int r=0; r<g_ab_.rows; r++) {

            g_ab_(r, r) = 0.f;
        }
        // make symmetrical
        for(int r=0; r<g_ab_.rows; r++) {

            for(int c=0; c<g_ab_.rows; c++) {

                g_ab_(r, c) = g_ab_(c, r);
            }
        }

        g_ij_ = Mat1f(I, I);
        g_ij_ = g_ab_.clone();    // make them equal
        Mat1f noise(g_ij_.size());
        randn(noise, 0.f, 0.02f);
        g_ij_ += noise;
        g_ij_.setTo(0.f, g_ij_ < 0.f);
        g_ij_.setTo(1.f, g_ij_ > 1.f);

        for(int r=0; r<g_ij_.rows; r++) {

            g_ij_(r, r) = 0.f;
            for(int c=0; c<g_ij_.cols; c++) {

                g_ij_(r, c) = g_ij_(c, r);
            }
        }

        // add test graphs to signal
        sig_.Append(NAME_GRAPH_AB, g_ab_);
        sig_.Append(NAME_GRAPH_IJ, g_ij_);
    }

    virtual void TearDown()
    {
        sig_.Clear();
    }

    // members
    LayerFactory::LayerShared to_; ///< pointer to test object

    Mat1f g_ab_;                 ///< adj. matrix for test graph
    Mat1f g_ij_;                 ///< adj. matrix for test graph

    Signal sig_;
};

TEST_F(GraphCompatibilityTest, Dims)
{
    for(int A=1; A<10; A++) {

        for(int I=1; I<10; I++) {

            // generate random adjacency matrices
            g_ab_ = Mat1f(A, A);
            Mat1i tmp(A, A);
            int stddev = 1000;
            randu(tmp, 0, stddev);
            g_ab_ = tmp/static_cast<float>(stddev);

            // distance from node to itself is 0
            for(int r=0; r<g_ab_.rows; r++) {

                g_ab_(r, r) = 0.f;
            }
            // make symmetrical
            for(int r=0; r<g_ab_.rows; r++) {

                for(int c=0; c<g_ab_.rows; c++) {

                    g_ab_(r, c) = g_ab_(c, r);
                }
            }

            g_ij_ = Mat1f(I, I);
            g_ij_ = g_ab_.clone();    // make them equal

            Signal sig;
            // add test graphs to signal
            sig.Append(NAME_GRAPH_AB, g_ab_);
            sig.Append(NAME_GRAPH_IJ, g_ij_);

            to_->Clear();
            to_->Activate(sig);
            to_->Response(sig);

            EXPECT_MAT_DIMS_EQ(sig.MostRecentMat(NAME_M),
                               Size2i(g_ab_.rows, g_ij_.rows))
                    << "Match matrix should be of size (A, I)";
        }
    }
}


} // annonymous namespace for test cases and fixtures

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

            sig_.Clear();
            // add test graphs to signal
            sig_.Append(NAME_GRAPH_AB, g_ab_);
            sig_.Append(NAME_GRAPH_IJ, g_ij_);

            to_->Clear();
            to_->Activate(sig_);
            to_->Response(sig_);

            EXPECT_MAT_DIMS_EQ(sig_.MostRecentMat(NAME_M),
                               Size2i(g_ab_.rows, g_ij_.rows))
                    << "Match matrix should be of size (A, I)";
        }
    }
}

/**
 * @brief graph without any connections should yield all-zero compatibility
 */
TEST_F(GraphCompatibilityTest, NoConnections)
{
    g_ab_ = Mat1f::zeros(5, 5);
    g_ij_ = g_ab_.clone();    // make them equal

    // add test graphs to signal
    sig_.Append(NAME_GRAPH_AB, g_ab_);
    sig_.Append(NAME_GRAPH_IJ, g_ij_);

    to_->Activate(sig_);
    to_->Response(sig_);

    EXPECT_MAT_EQ(sig_.MostRecentMat(NAME_M),
                  Mat1f::zeros(g_ab_.rows, g_ij_.rows));
}

/**
 * @brief zero-compatibility for node without edges
 */
TEST_F(GraphCompatibilityTest, NoConnections_for_one_node_in_g_ab)
{
    int vertex_idx = abs(randu<int>()) % g_ab_.rows;

    g_ab_.row(vertex_idx).setTo(0.f);
    g_ab_.col(vertex_idx).setTo(0.f);

    // add test graphs to signal
    sig_.Append(NAME_GRAPH_AB, g_ab_);
    sig_.Append(NAME_GRAPH_IJ, g_ij_);

    to_->Activate(sig_);
    to_->Response(sig_);

    Mat1f c_ai = sig_.MostRecentMat(NAME_M);

    EXPECT_MAT_EQ(c_ai.row(vertex_idx), Mat1f::zeros(1, g_ab_.cols))
            << "Expecting all-zero row for row i=" << vertex_idx
            << " since that vertex doesn't have any edges";
}

/**
 * @brief zero-compatibility for node without edges
 */
TEST_F(GraphCompatibilityTest, NoConnections_for_one_node_in_g_ij)
{
    int vertex_idx = abs(randu<int>()) % g_ij_.rows;

    g_ij_.row(vertex_idx).setTo(0.f);
    g_ij_.col(vertex_idx).setTo(0.f);

    // add test graphs to signal
    sig_.Append(NAME_GRAPH_AB, g_ab_);
    sig_.Append(NAME_GRAPH_IJ, g_ij_);

    to_->Activate(sig_);
    to_->Response(sig_);

    Mat1f c_ai = sig_.MostRecentMat(NAME_M);

    EXPECT_MAT_EQ(c_ai.col(vertex_idx), Mat1f::zeros(g_ij_.rows, 1))
            << "Expecting all-zero row for row i=" << vertex_idx
            << " since that vertex doesn't have any edges";
}

/**
 * @brief Make weights of two vertices very compatible and compare their
 * their compatibility score with that of other nodes.
 * This test does not assume that both adjacency graphs share the same dimensions.
 */
TEST_F(GraphCompatibilityTest, Compatibility)
{
    // sanity check
    ASSERT_GT(g_ab_.rows, 1) << "this test requires adjacency graphs for graphs with > 1 vertex";
    ASSERT_GT(g_ij_.rows, 1) << "this test requires adjacency graphs for graphs with > 1 vertex";

    const int MIN_DIM = static_cast<int>(min(g_ab_.rows, g_ij_.rows));

    const int N=5; // do this for multiple vertices
    for(int i=0; i<N; i++) {

        // make two vertices in both graph very compatible

        int vertex_compat_idx = abs(randu<int>()) % MIN_DIM;

        // actual weight value doesn't matter
        // it only matters that the edges in both graphs appear similar for a pair of vertices.
        int rand_value = static_cast<float>(abs(randu<int>()) % 100);
        float weight = rand_value/100.f;

        g_ab_.row(vertex_compat_idx).setTo(weight);
        g_ab_.col(vertex_compat_idx).setTo(weight);
        g_ab_(vertex_compat_idx, vertex_compat_idx) = 0.f;
        g_ij_.row(vertex_compat_idx).setTo(weight);
        g_ij_.col(vertex_compat_idx).setTo(weight);
        g_ij_(vertex_compat_idx, vertex_compat_idx) = 0.f;

        // make two other vertices in both graph very un-compatible
        int vertex_non_compat_idx = (vertex_compat_idx + 1) % MIN_DIM;

        g_ab_.row(vertex_non_compat_idx).setTo(weight);
        g_ab_.col(vertex_non_compat_idx).setTo(weight);

        // select second weight to be different from first for vertices to seem less compatible
        float weight2 = static_cast<float>(rand_value+30 % 100)/100.f;

        // sanity check
        ASSERT_GT(abs(weight-weight2), 0.25f) << "Difference between weights not large enough";

        g_ij_.row(vertex_non_compat_idx).setTo(weight2);
        g_ab_(vertex_non_compat_idx, vertex_non_compat_idx) = 0.f;
        g_ij_.col(vertex_non_compat_idx).setTo(weight2);
        g_ij_(vertex_non_compat_idx, vertex_non_compat_idx) = 0.f;

        // add test graphs to signal
        sig_.Append(NAME_GRAPH_AB, g_ab_);
        sig_.Append(NAME_GRAPH_IJ, g_ij_);

        to_->Activate(sig_);
        to_->Response(sig_);

        Mat1f c_ai = sig_.MostRecentMat(NAME_M);

        EXPECT_GT(c_ai(vertex_compat_idx, vertex_compat_idx), c_ai(vertex_non_compat_idx, vertex_non_compat_idx))
                << "Score of compatible vertex should exceed that of non-compatible one.";
    }
}

} // annonymous namespace for test cases and fixtures

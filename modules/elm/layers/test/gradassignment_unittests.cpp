#include "elm/layers/gradassignment.h"

#include <gtest/gtest.h>

#include <boost/graph/adjacency_list.hpp>

#include "elm/core/signal.h"
#include "elm/core/layerconfig.h"
#include "elm/layers/layerfactory.h"
#include "elm/ts/layer_assertions.h"

using namespace std;
using namespace boost;
using namespace cv;
using namespace elm;
namespace bg=boost::graph;

const float BETA            = 0.5f;
const float BETA_MAX        = 10.f;
const float BETA_RATE       = 1.075f;
const int MAX_ITER_PER_BETA = 4;
const int MAX_ITER_SINKHORN = 30;

const string NAME_GRAPH_AB = "G_ab";
const string NAME_GRAPH_IJ = "g_ij";
const string NAME_M        = "m";

// run standard/generalized layer tests
INSTANTIATE_TYPED_TEST_CASE_P(Layer_TP_GradAssignment_Test, Layer_TP_, GradAssignment);

class GradAssignmentTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        LayerConfig cfg;

        PTree p;
        p.put(GradAssignment::PARAM_BETA, BETA);
        p.put(GradAssignment::PARAM_BETA_MAX, BETA_MAX);
        p.put(GradAssignment::PARAM_BETA_RATE, BETA_RATE);
        p.put(GradAssignment::PARAM_MAX_ITER_PER_BETA, MAX_ITER_PER_BETA);
        p.put(GradAssignment::PARAM_MAX_ITER_SINKHORN, MAX_ITER_SINKHORN);

        cfg.Params(p);

        LayerIONames io;
        io.Input(GradAssignment::KEY_INPUT_GRAPH_AB, NAME_GRAPH_AB);
        io.Input(GradAssignment::KEY_INPUT_GRAPH_IJ, NAME_GRAPH_IJ);
        io.Output(GradAssignment::KEY_OUTPUT_M, NAME_M);

        to_ = LayerFactory::CreateShared("GradAssignment", cfg, io);

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
        randn(noise, 0.f, 0.5f);
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

    std::shared_ptr<base_Layer> to_; ///< ptr to test object

    Mat1f g_ab_;                 ///< adj. matrix for test graph
    Mat1f g_ij_;                 ///< adj. matrix for test graph

    Signal sig_;
};

TEST_F(GradAssignmentTest, ActivateAndResponse)
{
    to_->Activate(sig_);
    to_->Response(sig_);

    Mat1f m = sig_.MostRecentMat(NAME_M);

    EXPECT_MAT_DIMS_EQ(m, Size2i(g_ab_.rows, g_ij_.rows)) << "Match matrix should be of size (A, I)";

    // check we have a dominante diagonal
    for(int r=0; r<m.rows; r++) {

        for(int c=0; c<m.cols; c++) {

            if(r!=c) {

                EXPECT_LT(m(r, c), m(r, r)) << "Values > diagonal.";
            }
        }
    }

}

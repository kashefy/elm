/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/layers/gradassignment.h"

//#include <boost/graph/adjacency_list.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "elm/core/debug_utils.h"

#include "elm/core/signal.h"
#include "elm/core/layerconfig.h"
#include "elm/layers/graphcompatibility.h"
#include "elm/layers/layerfactory.h"
#include "elm/ts/layer_assertions.h"

using namespace std;
using namespace boost;
using namespace cv;
using namespace elm;
//namespace bg=boost::graph;

namespace {

ELM_INSTANTIATE_LAYER_TYPED_TEST_CASE_P(GradAssignment);

const float BETA            = 0.5f;
const float BETA_MAX        = 10.f;
const float BETA_RATE       = 1.075f;
const int MAX_ITER_PER_BETA = 4;
const int MAX_ITER_SINKHORN = 30;

const string NAME_GRAPH_AB          = "G_ab";
const string NAME_GRAPH_IJ          = "g_ij";
const string NAME_COMPATIBILITY_MAT = "c";
const string NAME_M                 = "m";

class GradAssignmentTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        {
            LayerConfig cfg;
            LayerIONames io;
            io.Input(GraphCompatibility::KEY_INPUT_GRAPH_AB, NAME_GRAPH_AB);
            io.Input(GraphCompatibility::KEY_INPUT_GRAPH_IJ, NAME_GRAPH_IJ);
            io.Output(GraphCompatibility::KEY_OUTPUT_RESPONSE, NAME_COMPATIBILITY_MAT);

            graph_compatibility_= LayerFactory::CreateShared("GraphCompatibility", cfg, io);
        }

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
            io.Input(GradAssignment::KEY_INPUT_MAT_COMPATIBILITY, NAME_COMPATIBILITY_MAT);

            io.Output(GradAssignment::KEY_OUTPUT_RESPONSE, NAME_M);

            to_ = LayerFactory::CreateShared("GradAssignment", cfg, io);
        }

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
        randn(noise, 0.f, 0.015f);
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

        graph_compatibility_->Activate(sig_);
        graph_compatibility_->Response(sig_);
    }

    virtual void TearDown()
    {
        sig_.Clear();
    }

    LayerFactory::LayerShared to_; ///< ptr to test object
    LayerFactory::LayerShared graph_compatibility_;

    Mat1f g_ab_;                 ///< adj. matrix for test graph
    Mat1f g_ij_;                 ///< adj. matrix for test graph

    Signal sig_;
};

TEST_F(GradAssignmentTest, ActivateAndResponse)
{
    to_->Activate(sig_);
    to_->Response(sig_);

    Mat1f m = sig_.MostRecentMat1f(NAME_M);

    EXPECT_MAT_DIMS_EQ(m, Size2i(g_ab_.rows, g_ij_.rows)) << "Match matrix should be of size (A, I)";

    // check we have a dominante diagonal
    for(int r=0; r<m.rows; r++) {

        for(int c=0; c<m.cols; c++) {

            if(r!=c) {

                EXPECT_LT(m(r, c), m(r, r)) << "Values > diagonal at r=" << r << ", c=" << c;
            }
        }
    }
}

TEST_F(GradAssignmentTest, ActivateAndResponse_large_graphs)
{
    // initialize test graphs
    const int A=30;  ///< no. of nodes in G
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

    // remove edges, large graphs are not expected to be fully connected, leads to overflow anyway.
    const int NB_NULL = static_cast<int>(0.5*A*A);
    for(int i=0; i<NB_NULL; i++) {

        int r = abs(randu<int>()) % A;
        int c = abs(randu<int>()) % A;
        g_ab_(r, c) = g_ab_(c, r) = 0.f;
    }

    g_ij_ = Mat1f(I, I);
    g_ij_ = g_ab_.clone();    // make them equal
    Mat1f noise(g_ij_.size());
    randn(noise, 0.f, 0.01f);
    //g_ij_ += noise;
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

    graph_compatibility_->Activate(sig_);
    graph_compatibility_->Response(sig_);

    to_->Activate(sig_);
    to_->Response(sig_);

    Mat1f m = sig_.MostRecentMat1f(NAME_M);

//    cout<<cv::format(m, cv::Formatter::FMT_NUMPY)<<std::endl;
//    cv::imshow("m/sum(m)", elm::ConvertTo8U(m/sum(m)[0]));
//    cv::waitKey();

    EXPECT_MAT_DIMS_EQ(m, Size2i(g_ab_.rows, g_ij_.rows)) << "Match matrix should be of size (A, I)";

    // check we have a dominante diagonal
    for(int r=0; r<m.rows; r++) {

        for(int c=0; c<m.cols; c++) {

            if(r!=c) {

                //EXPECT_LT(m(r, c), m(r, r)) << "Values > diagonal at r=" << r << ", c=" << c;
            }
        }
    }
}

TEST_F(GradAssignmentTest, Dims)
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

            graph_compatibility_->Activate(sig_);
            graph_compatibility_->Response(sig_);

            to_->Activate(sig_);
            to_->Response(sig_);

            EXPECT_MAT_DIMS_EQ(sig_.MostRecentMat1f(NAME_M),
                               Size2i(g_ab_.rows, g_ij_.rows))
                    << "Match matrix should be of size (A, I)";
        }
    }
}

/**
 * @brief Test that incrasing noise leads to deteriorated matching matrix
 *
 * For multiple iterations:
 *
 *  Measure difference between diagonal and other match variables.
 *  Sum difference across rows
 *
 * Measure slope across iterations, and see diagonal dominance decreasing
 *
 */
TEST_F(GradAssignmentTest, MoreNoise)
{
    g_ij_ = g_ab_.clone();    // start with equal graphs

    const int N=5;
    std::vector<float> diag_sum_diff(N);

    for(int i=0; i<N; i++) {

        // add noisy test graphs to signal
        sig_.Append(NAME_GRAPH_IJ, g_ij_);

        graph_compatibility_->Activate(sig_);
        graph_compatibility_->Response(sig_);

        to_->Activate(sig_);
        to_->Response(sig_);

        Mat1f m = sig_.MostRecentMat1f(NAME_M);

        // get diagonal values
        Mat1f diag(m.rows, 1);
        for(int r=0; r<m.rows; r++) {

            diag(r) = m(r, r);
        }

        diag_sum_diff[i] = sum(repeat(diag, 1, m.cols)-m)[0];

        // add more noise to g_ij_ for next iteration
        Mat1f noise(g_ij_.size());
        randn(noise, 0.f, 0.05f);
        g_ij_ += noise;
        g_ij_.setTo(0.f, g_ij_ < 0.f);
        g_ij_.setTo(1.f, g_ij_ > 1.f);

        // keep symmetric
        for(int r=0; r<g_ij_.rows; r++) {

            g_ij_(r, r) = 0.f;
            for(int c=0; c<g_ij_.cols; c++) {

                g_ij_(r, c) = g_ij_(c, r);
            }
        }
    }

    Mat _mean, _stddev;

    // Calculate mean slope to dermine if diagonals dominate less with noise.
    Mat1f deriv1;
    Sobel(diag_sum_diff, deriv1, CV_32F, 1, 0, 3);
    meanStdDev(deriv1, _mean, _stddev);

    EXPECT_LT(_mean.at<double>(), 0.) << "Diagonal dominance not decreasing with added noise." << Mat1f(diag_sum_diff).t();
}

} // annonymous namespace for test cases and fixtures


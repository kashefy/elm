/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/layers/graphcompatibility.h"

#include "elm/core/featuredata.h"
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

class GraphCompatibilityInitTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        LayerConfig cfg;

        LayerIONames io;
        io.Input(GraphCompatibility::KEY_INPUT_GRAPH_AB, NAME_GRAPH_AB);
        io.Input(GraphCompatibility::KEY_INPUT_GRAPH_IJ, NAME_GRAPH_IJ);
        io.Output(GraphCompatibility::KEY_OUTPUT_RESPONSE, NAME_M);

        to_ = LayerFactory::CreateShared("GraphCompatibility", cfg, io);
    }

    // members
    LayerFactory::LayerShared to_; ///< pointer to test object
};

class GraphCompatibilityTest : public GraphCompatibilityInitTest
{
protected:
    virtual void SetUp()
    {
        GraphCompatibilityInitTest::SetUp();

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
    Mat1f g_ab_;                 ///< adj. matrix for test graph
    Mat1f g_ij_;                 ///< adj. matrix for test graph

    Signal sig_;
};

TEST_F(GraphCompatibilityTest, Activate_invalid_non_square_adj_mat)
{
    g_ab_ = Mat1f::ones(11, 7) - Mat1f::eye(11, 7);
    g_ij_ = g_ab_.clone();

    sig_.Append(NAME_GRAPH_AB, g_ab_);
    sig_.Append(NAME_GRAPH_IJ, g_ij_);

    EXPECT_THROW(to_->Activate(sig_), ExceptionBadDims);
}

TEST_F(GraphCompatibilityTest, Dims)
{
    for(int A=1; A<10; A++) {

        for(int I=1; I<10; I++) {

            // generate random adjacency matrices
            g_ab_ = Mat1f(A, A, 1.f);

            g_ij_ = Mat1f(I, I, 1.f);

            sig_.Clear();
            // add test graphs to signal
            sig_.Append(NAME_GRAPH_AB, g_ab_);
            sig_.Append(NAME_GRAPH_IJ, g_ij_);

            to_->Clear();
            to_->Activate(sig_);
            to_->Response(sig_);

            SparseMat1f c_aibj = sig_.MostRecent(NAME_M).get<SparseMat1f>();
            ASSERT_EQ(4, c_aibj.dims()) << "Match matrix should be of size (A, I, A, I)";

            EXPECT_EQ(A, c_aibj.size(0)) << "Match matrix should be of size (A, I, A, I)";
            EXPECT_EQ(I, c_aibj.size(1)) << "Match matrix should be of size (A, I, A, I)";
            EXPECT_EQ(A, c_aibj.size(2)) << "Match matrix should be of size (A, I, A, I)";
            EXPECT_EQ(I, c_aibj.size(3)) << "Match matrix should be of size (A, I, A, I)";
        }
    }
}

/**
 * @brief graph without any connections should yield all-zero compatibility
 */
TEST_F(GraphCompatibilityTest, NoConnections)
{
    const int NB_VERTICES = 5;
    g_ab_ = Mat1f::zeros(NB_VERTICES, NB_VERTICES);
    g_ij_ = g_ab_.clone();    // make them equal

    // add test graphs to signal
    sig_.Append(NAME_GRAPH_AB, g_ab_);
    sig_.Append(NAME_GRAPH_IJ, g_ij_);

    to_->Activate(sig_);
    to_->Response(sig_);

    SparseMat1f c_aibj = sig_.MostRecent(NAME_M).get<SparseMat1f>();
    ASSERT_EQ(4, c_aibj.dims()) << "Match matrix should be of size (A, I, A, I)";

    EXPECT_EQ(NB_VERTICES, c_aibj.size(0)) << "Match matrix should be of size (A, I, A, I)";
    EXPECT_EQ(NB_VERTICES, c_aibj.size(1)) << "Match matrix should be of size (A, I, A, I)";
    EXPECT_EQ(NB_VERTICES, c_aibj.size(2)) << "Match matrix should be of size (A, I, A, I)";
    EXPECT_EQ(NB_VERTICES, c_aibj.size(3)) << "Match matrix should be of size (A, I, A, I)";

    EXPECT_EQ(static_cast<size_t>(0), c_aibj.nzcount());
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

    SparseMat1f c_aibj = sig_.MostRecent(NAME_M).get<SparseMat1f>();

    for(int i=0; i<g_ij_.rows; i++) {

        for(int b=0; b<g_ab_.rows; b++) {

            for(int j=0; j<g_ij_.rows; j++) {

                int idx[4] = {vertex_idx, i, b, j};
                EXPECT_FLOAT_EQ(0.f, c_aibj.ref(idx))
                        << "Expecting all-zero row for slice a=" << vertex_idx
                        << " since this vertex doesn't have any edges";
            }
        }
    }
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

    SparseMat1f c_aibj = sig_.MostRecent(NAME_M).get<SparseMat1f>();

    for(int a=0; a<g_ij_.rows; a++) {

        for(int b=0; b<g_ab_.rows; b++) {

            for(int j=0; j<g_ij_.rows; j++) {

                int idx[4] = {a, vertex_idx, b, j};
                EXPECT_FLOAT_EQ(0.f, c_aibj.ref(idx))
                        << "Expecting all-zero row for slice i=" << vertex_idx
                        << " since this vertex doesn't have any edges";
            }
        }
    }
}

/**
 * @brief Make weights of two vertices very compatible and compare
 * their compatibility score with that of other nodes.
 * This test does not assume that both adjacency graphs share the same dimensions.
 */
TEST_F(GraphCompatibilityTest, CompatibilityIntegration)
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

        SparseMat1f c_aibj = sig_.MostRecent(NAME_M).get<SparseMat1f>();
        Mat1f c_ai = GraphCompatibility::Integrate(c_aibj, Mat1f(g_ab_.rows, g_ij_.rows, 1.f));

        EXPECT_GT(c_ai(vertex_compat_idx, vertex_compat_idx), c_ai(vertex_non_compat_idx, vertex_non_compat_idx))
                << "Score of compatible vertex should exceed that of non-compatible one.";
    }
}

TEST(GraphCompatibilityStaticTest, Integrate_invalid_dims)
{
    Mat1f tmp(3, 2, 0.5f);

    SparseMat1f s(tmp);

    {
        Mat1f m(s.size(0), s.size(1), 1.f);
        EXPECT_NO_THROW(GraphCompatibility::Integrate(s, m));
    }

    {
        Mat1f m(s.size(0)+1, s.size(1), 1.f);
        EXPECT_THROW(GraphCompatibility::Integrate(s, m), ExceptionBadDims);
    }

    {
        Mat1f m(s.size(0), s.size(1)+1, 1.f);
        EXPECT_THROW(GraphCompatibility::Integrate(s, m), ExceptionBadDims);
    }
}

class GraphCompatibilityProtected : public GraphCompatibility
{
public:
    float Compatibility(float w1, float w2) const
    {
        return GraphCompatibility::Compatibility(w1, w2);
    }
};

class GraphCompatibilityProtectedTest : public ::testing::Test
{
};

TEST_F(GraphCompatibilityProtectedTest, Compatibility_weights)
{
    GraphCompatibilityProtected to; ///< test object
    EXPECT_EQ(0.f, to.Compatibility(0.f, 1.f));
    EXPECT_EQ(0.f, to.Compatibility(0.f, 3.f));
    EXPECT_EQ(to.Compatibility(1.f, 0.f), to.Compatibility(0.f, 1.f)) << "compatibility is not symmetric";
    EXPECT_EQ(to.Compatibility(0.3f, 0.f), to.Compatibility(0.f, 0.3f)) << "compatibility is not symmetric";
    EXPECT_EQ(to.Compatibility(0.7, 0.3), to.Compatibility(0.3f, 0.7f)) << "compatibility is not symmetric";
}

} // annonymous namespace for test cases and fixtures

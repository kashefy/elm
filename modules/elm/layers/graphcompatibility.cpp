/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/layers/graphcompatibility.h"

#include "elm/core/debug_utils.h"
#include "elm/core/exception.h"
#include "elm/core/inputname.h"
#include "elm/core/layerconfig.h"
#include "elm/core/signal.h"
#include "elm/layers/sinkhornbalancing.h"
#include "elm/ts/layerattr_.h"

using namespace std;
using namespace cv;
using namespace elm;

// I/O Names
const string GraphCompatibility::KEY_INPUT_GRAPH_AB = "G_ab";
const string GraphCompatibility::KEY_INPUT_GRAPH_IJ = "g_ij";

/** @todo why does define guard lead to undefined reference error?
 */
//#ifdef __WITH_GTEST
#include <boost/assign/list_of.hpp>
template <>
elm::MapIONames LayerAttr_<GraphCompatibility>::io_pairs = boost::assign::map_list_of
        ELM_ADD_INPUT_PAIR(GraphCompatibility::KEY_INPUT_GRAPH_AB)
        ELM_ADD_INPUT_PAIR(GraphCompatibility::KEY_INPUT_GRAPH_IJ)
        ELM_ADD_OUTPUT_PAIR(elm::detail::BASE_SPARSEMATOUTPUT_LAYER__KEY_OUTPUT_RESPONSE)
        ;
//#endif

GraphCompatibility::~GraphCompatibility()
{
}

GraphCompatibility::GraphCompatibility()
    : base_SparseMatOutputLayer()
{
    Clear();
}

GraphCompatibility::GraphCompatibility(const LayerConfig &cfg)
    : base_SparseMatOutputLayer(cfg)
{
    Reset(cfg);
}

void GraphCompatibility::Clear()
{
    m_.clear();
}

void GraphCompatibility::Reset(const LayerConfig &config)
{
    Clear();
    Reconfigure(config);
}

void GraphCompatibility::Reconfigure(const LayerConfig &config)
{
}

void GraphCompatibility::InputNames(const LayerInputNames &io)
{
    name_g_ab_ = io.Input(KEY_INPUT_GRAPH_AB);
    name_g_ij_ = io.Input(KEY_INPUT_GRAPH_IJ);
}

void GraphCompatibility::Activate(const Signal &signal)
{
    Mat1f g_ab = signal.MostRecentMat1f(name_g_ab_);
    Mat1f g_ij = signal.MostRecentMat1f(name_g_ij_);

    if(g_ab.rows != g_ab.cols) {

        ELM_THROW_BAD_DIMS("G_ab's adjacency matrix is not a square matrix.");
    }

    if(g_ij.rows != g_ij.cols) {

        ELM_THROW_BAD_DIMS("G_ij's adjacency matrix is not a square matrix.");
    }

    A_ = g_ab.rows; ///< no. of vertices in G_ab graph
    I_ = g_ij.rows; ///< no. of vertices in G_ij graph

    m_ = Compatibility(g_ab, g_ij);
}

Mat1f GraphCompatibility::Integrate(SparseMat1f &c_aibj, const Mat1f &m)
{
    int A = c_aibj.size(0);
    int I = c_aibj.size(1);

    ELM_THROW_BAD_DIMS_IF(m.rows != A,
                          "No. of mat. rows must match 1st dimension in compat. matrix.");

    ELM_THROW_BAD_DIMS_IF(m.cols != I,
                          "No. of mat. cols must match 2nd dimension in compat. matrix.");

    const int DIMS = 4;
    Mat1f c_ai(A, I);

    int idx[DIMS];
    int &a = idx[0];
    int &i = idx[1];
    int &b = idx[2];
    int &j = idx[3];

    for(a=0; a<A; a++) {

        for(i=0; i<I; i++) {

            float sigma_over_bj = 0.f;

            for(b=0; b<A; b++) {

                // skip for zero-weighted edges.
                for(j=0; j<I; j++) {

                    sigma_over_bj += c_aibj.ref(idx) * m(a, i);
                } // j
            } // b

            c_ai(a, i) = sigma_over_bj;
        } // i
    } // a

    return c_ai;
}

SparseMat1f GraphCompatibility::Compatibility(const Mat1f &g_ab, const Mat1f &g_ij) const
{
    const int DIMS = 4;
    int _size[DIMS] = {A_, I_, A_, I_};
    SparseMat1f c_aibj(DIMS, _size);

    int idx[DIMS];
    int &a = idx[0];
    int &i = idx[1];
    int &b = idx[2];
    int &j = idx[3];

    for(a=0; a<A_; a++) {

        Mat1f g_ab_row = g_ab.row(a);

        for(i=0; i<I_; i++) {

            Mat1f g_ij_row = g_ij.row(i);

            for(b=0; b<A_; b++) {

                float g_ab_row_at_b = g_ab_row(b);

                // skip for zero-weighted edges.
                if(g_ab_row_at_b != 0.f) {

                    for(j=0; j<I_; j++) {

                        float g_ij_row_at_j = g_ij_row(j);

                        // skip for zero-weighted edges.
                        if(g_ij_row_at_j != 0.f) {

                            float compatibility = Compatibility(g_ab_row_at_b, g_ij_row_at_j);
                            c_aibj.ref(idx) = compatibility;
                        }
                    } // j
                }
            } // b
        } // i
    } // a

    return c_aibj;
}

float GraphCompatibility::Compatibility(float w1, float w2) const
{
    float c;
    if(w1 == 0.f || w2 == 0.f) {

        c = 0.f;
    }
    else {
        //c = w1*w2;
        c = 1.f-3.f*abs(w1-w2);
    }
    return c;
}

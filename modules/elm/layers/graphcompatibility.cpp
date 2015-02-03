/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, Elm Library Project
// 3-clause BSD License
//
//M*/
#include "elm/layers/graphcompatibility.h"

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
        ELM_ADD_OUTPUT_PAIR(detail::BASE_MATOUTPUT_LAYER__KEY_OUTPUT_RESPONSE)
        ;
//#endif

GraphCompatibility::~GraphCompatibility()
{
}

GraphCompatibility::GraphCompatibility()
    : base_MatOutputLayer()
{
    Clear();
}

GraphCompatibility::GraphCompatibility(const LayerConfig &cfg)
    : base_MatOutputLayer(cfg)
{
    Reset(cfg);
}

void GraphCompatibility::Clear()
{
    m_ = Mat1f();
}

void GraphCompatibility::Reset(const LayerConfig &config)
{
    Clear();
    Reconfigure(config);
}

void GraphCompatibility::Reconfigure(const LayerConfig &config)
{
}

void GraphCompatibility::IONames(const LayerIONames &io)
{
    base_MatOutputLayer::IONames(io);
    name_g_ab_ = io.Input(KEY_INPUT_GRAPH_AB);
    name_g_ij_ = io.Input(KEY_INPUT_GRAPH_IJ);
}

void GraphCompatibility::Activate(const Signal &signal)
{
    Mat1f g_ab = signal.MostRecentMat(name_g_ab_);
    Mat1f g_ij = signal.MostRecentMat(name_g_ij_);

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

Mat1f GraphCompatibility::Compatibility(const Mat1f &g_ab, const Mat1f &g_ij) const
{
    Mat1f c_ai(A_, I_, 0.f);
    for(int a=0; a<A_; a++) {

        Mat1f g_ab_row = g_ab.row(a);

        for(int i=0; i<I_; i++) {

            Mat1f g_ij_row = g_ij.row(i);

            float sigma_c_aibj_over_bj = 0.f;
            for(size_t b=0; b<g_ab_row.total(); b++) {

                float g_ab_row_at_b = g_ab_row(b);

                // skip for zero-weighted edges.
                if(g_ab_row_at_b != 0.f) {

                    for(size_t j=0; j<g_ij_row.total(); j++) {

                        sigma_c_aibj_over_bj += Compatibility(g_ab_row_at_b, g_ij_row(j));
                    }
                }
            }

            c_ai(a, i) = sigma_c_aibj_over_bj;
        }
    }

    //c_ai /= static_cast<float>(max(A_, I_));

    return c_ai;
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

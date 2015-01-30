/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, Elm Library Project
// 3-clause BSD License
//
//M*/
#include "elm/layers/gradassignment.h"

#include "elm/core/layerconfig.h"
#include "elm/core/signal.h"
#include "elm/layers/sinkhornbalancing.h"
#include "elm/ts/layerattr_.h"

using namespace std;
using namespace cv;
using namespace elm;

// params
const string GradAssignment::PARAM_BETA                 = "beta0";
const string GradAssignment::PARAM_BETA_MAX             = "beta_max";
const string GradAssignment::PARAM_BETA_RATE            = "beta_rate";
const string GradAssignment::PARAM_MAX_ITER_PER_BETA    = "max_iter_per_beta";
const string GradAssignment::PARAM_MAX_ITER_SINKHORN    = "max_iter_sinkhorn";

// I/O Names
const string GradAssignment::KEY_INPUT_GRAPH_AB = "G_ab";
const string GradAssignment::KEY_INPUT_GRAPH_IJ = "g_ij";
const string GradAssignment::KEY_OUTPUT_M       = "m";

const float GradAssignment::EPSILON = 1e-2;

/** @todo why does define guard lead to undefined reference error?
 */
//#ifdef __WITH_GTEST
#include <boost/assign/list_of.hpp>
template <>
elm::MapIONames LayerAttr_<GradAssignment>::io_pairs = boost::assign::map_list_of
        ELM_ADD_INPUT_PAIR(GradAssignment::KEY_INPUT_GRAPH_AB)
        ELM_ADD_INPUT_PAIR(GradAssignment::KEY_INPUT_GRAPH_IJ)
        ELM_ADD_OUTPUT_PAIR(GradAssignment::KEY_OUTPUT_M)
        ;
//#endif

GradAssignment::~GradAssignment()
{
}

GradAssignment::GradAssignment()
    : base_Layer()
{
    Clear();
}

GradAssignment::GradAssignment(const LayerConfig &cfg)
    : base_Layer(cfg)
{
    Reset(cfg);
}

void GradAssignment::Clear()
{
    m_ai_ = Mat1f();
}

void GradAssignment::Reset(const LayerConfig &config)
{
    Clear();
    Reconfigure(config);
}

void GradAssignment::Reconfigure(const LayerConfig &config)
{
    PTree params = config.Params();

    beta_0_ = params.get<float>(PARAM_BETA);
    if(beta_0_ <= 0.f) {

        ELM_THROW_VALUE_ERROR("Control parameter beta must be positive.");
    }

    beta_max_ = params.get<float>(PARAM_BETA_MAX);

    beta_rate_ = params.get<float>(PARAM_BETA_RATE);
    if(beta_rate_ <= 1.f) {

        ELM_THROW_VALUE_ERROR("rate must be > 1.");
    }

    max_iter_per_beta_ = params.get<int>(PARAM_MAX_ITER_SINKHORN);
    max_iter_sinkhorn_ = params.get<int>(PARAM_MAX_ITER_PER_BETA);
}

void GradAssignment::IONames(const LayerIONames &config)
{
    name_g_ab_ = config.Input(KEY_INPUT_GRAPH_AB);
    name_g_ij_ = config.Input(KEY_INPUT_GRAPH_IJ);
    name_out_m_ = config.Output(KEY_OUTPUT_M);
}

void GradAssignment::Activate(const Signal &signal)
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

    Mat1f c_ai = Compatibility(g_ab, g_ij);

    bool is_m_converged = false;

    Mat1f m_ai_hat = Mat1f(A_+1, I_+1, 1.f+EPSILON); // add slack variables to be more robust to outliers

    m_ai_ = m_ai_hat(Rect2i(0, 0, A_, I_));

    float beta = beta_0_;

    int nb_iterations_0 = 0;

    while(beta < beta_max_) { // A

        nb_iterations_0 = 0;
        while(!is_m_converged && nb_iterations_0 < max_iter_per_beta_) { // B

            Mat1f m_ai_hat_0 = m_ai_hat.clone();

            Mat1f q_ai(A_, I_); ///< partial derivative of E_wg with respect to M_ai

            multiply(c_ai, m_ai_, q_ai);
            // softassign
            exp(beta*q_ai, m_ai_);

            is_m_converged = SinkhornBalancing::RowColNormalization(m_ai_hat, max_iter_sinkhorn_, EPSILON); // C

            is_m_converged = sum(abs(m_ai_hat-m_ai_hat_0))[0] < EPSILON;

            nb_iterations_0++;
        } // end B

        beta *= beta_rate_;
    } // end A

}

void GradAssignment::Response(Signal &signal)
{
    signal.Append(name_out_m_, m_ai_);
}

Mat1f GradAssignment::Compatibility(const Mat1f &g_ab, const Mat1f &g_ij) const
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

float GradAssignment::Compatibility(float w1, float w2) const
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

/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/layers/gradassignment.h"

#include "elm/core/debug_utils.h"

#include "elm/core/inputname.h"
#include "elm/core/layerconfig.h"
#include "elm/core/signal.h"
#include "elm/layers/graphcompatibility.h"
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
const string GradAssignment::KEY_INPUT_MAT_COMPATIBILITY = "c_aibj";

const float GradAssignment::EPSILON = 1e-2;

/** @todo why does define guard lead to undefined reference error?
 */
//#ifdef __WITH_GTEST
#include <boost/assign/list_of.hpp>
template <>
elm::MapIONames LayerAttr_<GradAssignment>::io_pairs = boost::assign::map_list_of
        ELM_ADD_INPUT_PAIR(GradAssignment::KEY_INPUT_GRAPH_AB)
        ELM_ADD_INPUT_PAIR(GradAssignment::KEY_INPUT_GRAPH_IJ)
        ELM_ADD_INPUT_PAIR(GradAssignment::KEY_INPUT_MAT_COMPATIBILITY)
        ELM_ADD_OUTPUT_PAIR(detail::BASE_MATOUTPUT_LAYER__KEY_OUTPUT_RESPONSE)
        ;
//#endif

GradAssignment::~GradAssignment()
{
}

GradAssignment::GradAssignment()
    : base_MatOutputLayer()
{
    Clear();
}

GradAssignment::GradAssignment(const LayerConfig &cfg)
    : base_MatOutputLayer(cfg)
{
    Reset(cfg);
}

void GradAssignment::Clear()
{
    m_ = Mat1f();
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

void GradAssignment::InputNames(const LayerInputNames &io)
{
    name_g_ab_ = io.Input(KEY_INPUT_GRAPH_AB);
    name_g_ij_ = io.Input(KEY_INPUT_GRAPH_IJ);
    name_c_ai_ = io.Input(KEY_INPUT_MAT_COMPATIBILITY);
}

void GradAssignment::Activate(const Signal &signal)
{
    // get inputs
    Mat1f g_ab = signal.MostRecentMat1f(name_g_ab_);
    Mat1f g_ij = signal.MostRecentMat1f(name_g_ij_);

    if(g_ab.rows != g_ab.cols) {

        ELM_THROW_BAD_DIMS("G_ab's adjacency matrix is not a square matrix.");
    }

    if(g_ij.rows != g_ij.cols) {

        ELM_THROW_BAD_DIMS("g_ij's adjacency matrix is not a square matrix.");
    }

    A_ = g_ab.rows; ///< no. of vertices in G_ab graph
    I_ = g_ij.rows; ///< no. of vertices in G_ij graph

    SparseMat1f c_aibj = signal.MostRecentMat1f(name_c_ai_);//Compatibility(g_ab, g_ij);

    ELM_THROW_BAD_DIMS_IF(c_aibj.dims() != 4, "Expecting a 4-dim compatibility matrix for 2 graphs.");

    ELM_THROW_BAD_DIMS_IF(c_aibj.size(0) != A_,
                          "Size of 1st dim. != no. of vertices in 1st graph.");

    ELM_THROW_BAD_DIMS_IF(c_aibj.size(2) != A_,
                          "Size of 3rd dim. != no. of vertices in 1st graph.");

    ELM_THROW_BAD_DIMS_IF(c_aibj.size(1) != I_,
                          "Size of 2nd dim. != no. of vertices in 2nd graph.");

    ELM_THROW_BAD_DIMS_IF(c_aibj.size(3) != I_,
                          "Size of 4th dim. != no. of vertices in 2nd graph.");

    // got all inputs, now go go go
    bool is_m_converged = false;

    Mat1f m_ai_hat = Mat1f(A_+1, I_+1, 1.f+EPSILON); // add slack variables to be more robust to outliers

    m_ = m_ai_hat(Rect2i(0, 0, A_, I_)); // initialize match matrix variables.

    float beta = beta_0_;

    int nb_iterations_0;

    //ELM_COUT_VAR(cv::format(c_ai, Formatter::FMT_NUMPY));
    while(beta < beta_max_) { // A

        nb_iterations_0 = 0;
        while(!is_m_converged && nb_iterations_0 < max_iter_per_beta_) { // B

            Mat1f m_ai_hat_0 = m_ai_hat.clone();

            //ELM_COUT_VAR(m_ai_hat_0);
//            Mat1f q_ai = q_ai(A_, I_); ///< partial derivative of E_wg with respect to M_ai

//            multiply(c_ai, m_, q_ai);
            Mat1f q_ai = GraphCompatibility::Integrate(c_aibj, m_);

            //imshow("m", ConvertTo8U(m_));
            //cv::waitKey();

            //ELM_COUT_VAR(q_ai);
            // softassign
            exp(beta*q_ai, m_);
            //ELM_COUT_VAR(m_);

            is_m_converged = SinkhornBalancing::RowColNormalization(m_ai_hat, max_iter_sinkhorn_, 0.05f); // C

            //ELM_COUT_VAR(m_ai_hat);
            is_m_converged = sum(abs(m_-m_ai_hat_0(Rect2i(0, 0, A_, I_))))[0] < 0.5f;

            nb_iterations_0++;
        } // end B

        beta *= beta_rate_;
    } // end A

}


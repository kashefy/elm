#include "elm/layers/gradassignment.h"

#include "elm/core/layerconfig.h"
#include "elm/core/signal.h"

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
    m_ai = Mat1f();
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
    Mat1f g_ij = signal.MostRecentMat(name_g_ab_);

    if(g_ab.rows != g_ab.cols) {

        ELM_THROW_BAD_DIMS("G_ab's adjacency matrix is not a square matrix.");
    }

    if(g_ij.rows != g_ij.cols) {

        ELM_THROW_BAD_DIMS("G_ij's adjacency matrix is not a square matrix.");
    }

    A = g_ab.rows; ///< no. of vertices in G_ab graph
    I = g_ij.rows; ///< no. of vertices in G_ij graph

    Mat1f c_ai = CompatibilityMat(g_ab, g_ij);

    bool is_m_converged = false;


    m_ai = Mat1f(A, I, 1.f+EPSILON);
    float beta = beta_0_;

    int nb_iterations_0 = 0;
    int nb_iterations_1 = 0;

    while(beta < beta_max_) { // A

        nb_iterations_0 = 0;
        while(!is_m_converged && nb_iterations_0 < max_iter_per_beta_) { // B

            //std::cout<<"B"<<nb_iterations_0<<std::endl;
            Mat1f q_ai;      ///< partial derivative of Ewg with respect to M_ai

            // c_aibj
//            Mat1f c_aibj;
//            Mat1f i_sigma_g_ab_over_b;
//            reduce(g_ab, i_sigma_g_ab_over_b, 0, cv::REDUCE_SUM);
//            i_sigma_g_ab_over_b *= I;

//            Mat1f a_sigma_g_ij_over_j;
//            reduce(g_ij, a_sigma_g_ij_over_j, 1, cv::REDUCE_SUM);
//            a_sigma_g_ij_over_j *= A;

//            c_aibj = a_sigma_g_ij_over_j*i_sigma_g_ab_over_b;
//            std::cout<<"c_aibj="<<std::endl<<c_aibj<<std::endl;
            // c_aibj

            // sum_bj_Mbj
            float sum_bj_Mbj = cv::sum(m_ai)[0];
            //reduce(m_ai, sum_bj_Mbj, 1, cv::REDUCE_SUM);

            // sum_bj_Mbj
            // q_ai
            //cv::multiply(c_aibj, sum_bj_Mbj, q_ai);
            cv::multiply(c_ai, sum_bj_Mbj, q_ai);

            //std::cout<<"h1"<<std::endl;
            // q_ai
            // softassign
            Mat1f m_ai0;
            cv::exp(beta*q_ai, m_ai0);

            //Mat1f m_ai_hat0 = m_ai_hat.clone();    ///< match matrix variables including the slacks

            //is_m_hat_converged = false;
            nb_iterations_1 = 0;
            //while(!is_m_hat_converged && nb_iterations_1 < MAX_ITERATIONS_1) { // C
            while(!is_m_converged && nb_iterations_1 < max_iter_sinkhorn_) { // C

                //std::cout<<"C"<<nb_iterations_1<<std::endl;

                // update m by normalizing across all rows
                Mat1f row_sums_i;
                reduce(m_ai0, row_sums_i, 1, cv::REDUCE_SUM);
                row_sums_i = cv::repeat(row_sums_i, 1, m_ai0.cols);
                Mat1f m_ai1 = m_ai0/row_sums_i;

                // update m by normalizing across all columns
                Mat1f col_sums_i;
                reduce(m_ai1, col_sums_i, 0, cv::REDUCE_SUM);
                col_sums_i = cv::repeat(col_sums_i, m_ai0.rows, 1);
                m_ai0 = m_ai1/col_sums_i;

                nb_iterations_1++;

//                std::cout<<"m_ai"<<std::endl;
//                std::cout<<m_ai<<std::endl;
//                std::cout<<"m_ai0"<<std::endl;
//                std::cout<<m_ai0<<std::endl;
                is_m_converged = cv::sum(abs(m_ai-m_ai0))[0] < EPSILON;

                m_ai = m_ai0;

            } // end C

            nb_iterations_0++;

//            std::cout<<"m_ai"<<std::endl;
//            std::cout<<m_ai<<std::endl;
//            std::cout<<"m_ai0"<<std::endl;
//            std::cout<<m_ai0<<std::endl;
            //is_m_converged = cv::sum(abs(m_ai-m_ai0))[0] < epislon;

        } // end B

        beta *= beta_rate_;
    } // end A

    std::cout<<"m_ai"<<std::endl;
    std::cout<<m_ai<<std::endl;

}

void GradAssignment::Response(Signal &signal)
{
    signal.Append(name_out_m_, m_ai);
}

Mat1f GradAssignment::CompatibilityMat(const Mat1f &g_ab, const Mat1f &g_ij) const
{
    Mat1f c_ai(A, I, 0.f);
    for(int a=0; a<A; a++) {

        Mat1f g_ab_row = g_ab.row(a);
        for(int i=0; i<I; i++) {

            Mat1f g_ij_row = g_ij.row(i);

            float sigma_c_aibj_over_bj = 0.f;
            for(size_t b=0; b<g_ab_row.total(); b++) {

                for(size_t j=0; j<g_ij_row.total(); j++) {

                    sigma_c_aibj_over_bj += CompatibilityFunc(g_ab_row(b), g_ij_row(j));
                }
            }

            c_ai(a, i) = sigma_c_aibj_over_bj;
        }
    }

    return c_ai;
}

float GradAssignment::CompatibilityFunc(float w1, float w2) const
{
    float c;
    if(w1 == 0.f || w2 == 0.f) {

        c = 0.f;
    }
    else {
        c = 1-3.f*abs(w1-w2);
    }
    return c;
}

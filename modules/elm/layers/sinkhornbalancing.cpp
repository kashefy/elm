/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, Elm Library Project
// 3-clause BSD License
//
//M*/
#include "elm/layers/sinkhornbalancing.h"

#include "elm/core/layerconfig.h"
#include "elm/core/signal.h"
#include "elm/ts/layerattr_.h"

using namespace std;
using namespace cv;
using namespace elm;

const string SinkhornBalancing::PARAM_EPSILON   = "epsilon";
const string SinkhornBalancing::PARAM_MAX_ITER  = "max_iter";

const string SinkhornBalancing::KEY_INPUT_MAT           = "mat_in";
const string SinkhornBalancing::KEY_OUTPUT_MAT_BALANCED = "mat_balanced";
const string SinkhornBalancing::KEY_OUTPUT_IS_CONVERGED = "convergence";

/** @todo why does define guard lead to undefined reference error?
 */
//#ifdef __WITH_GTEST
#include <boost/assign/list_of.hpp>
template <>
elm::MapIONames LayerAttr_<SinkhornBalancing>::io_pairs = boost::assign::map_list_of
        ELM_ADD_INPUT_PAIR(SinkhornBalancing::KEY_INPUT_MAT)
        ELM_ADD_OUTPUT_PAIR(SinkhornBalancing::KEY_OUTPUT_MAT_BALANCED)
        ELM_ADD_OUTPUT_PAIR(SinkhornBalancing::KEY_OUTPUT_IS_CONVERGED)
        ;
//#endif

SinkhornBalancing::SinkhornBalancing()
    : base_Layer()
{
    Clear();
}

SinkhornBalancing::SinkhornBalancing(const LayerConfig &cfg)
    : base_Layer(cfg)
{
    Reset(cfg);
}

void SinkhornBalancing::Clear()
{
    m_ = Mat1f();
}

void SinkhornBalancing::Reset(const LayerConfig &config)
{
    Clear();
    Reconfigure(config);
}

void SinkhornBalancing::Reconfigure(const LayerConfig &config)
{
    PTree params = config.Params();

    epsilon_ = params.get<float>(PARAM_EPSILON);
    max_iter_ = params.get<int>(PARAM_MAX_ITER);
}

void SinkhornBalancing::IONames(const LayerIONames &io)
{
    name_in_m_              = io.Input(KEY_INPUT_MAT);
    name_out_convergence_   = io.Output(KEY_OUTPUT_IS_CONVERGED);
    name_out_m_             = io.Output(KEY_OUTPUT_MAT_BALANCED);
}

void SinkhornBalancing::Activate(const Signal &signal)
{
    m_ = signal.MostRecentMat(name_in_m_);

    is_converged = RowColNormalization(m_, max_iter_, epsilon_);
}

void SinkhornBalancing::Response(Signal &signal)
{
    signal.Append(name_out_m_, m_);
    signal.Append(name_out_convergence_, Mat1f(1, 1, static_cast<float>(is_converged)));
}

bool SinkhornBalancing::RowColNormalization(Mat1f &m, int max_iter, float epsilon)
{
    bool is_m_converged = false;

    if(m.empty()) {

        return is_m_converged;
    }

    int i = 0;

    // begin C
    while(!is_m_converged && i < max_iter) {

        // update m by normalizing across all rows
        Mat1f row_sums_i;
        reduce(m, row_sums_i, 1, REDUCE_SUM);
        row_sums_i = repeat(row_sums_i, 1, m.cols);
        Mat1f m_ai1 = m/row_sums_i;

        // update m by normalizing across all columns
        Mat1f col_sums_i;
        reduce(m_ai1, col_sums_i, 0, REDUCE_SUM);
        col_sums_i = repeat(col_sums_i, m_ai1.rows, 1);

        Mat1f tmp = m_ai1/col_sums_i;

        is_m_converged = sum(abs(tmp-m))[0] < epsilon;
        m = tmp;

        i++;
    } // end C

    return is_m_converged;
}

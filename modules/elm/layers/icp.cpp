/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/layers/icp.h"

#ifdef __WITH_PCL   // the layer is otherwise implemented as unsupported

#include <pcl/registration/icp.h>

#include <opencv2/core/eigen.hpp> // for eigen2cv(), must be preceeded definitio of Eigen either PCL or #include <Eigen/Dense>

#include "elm/core/exception.h"
#include "elm/core/inputname.h"
#include "elm/core/layerconfig.h"
#include "elm/core/signal.h"
#include "elm/ts/layerattr_.h"

using namespace std;
using namespace cv;
using namespace pcl;
using namespace elm;

typedef IterativeClosestPoint<PointXYZ, PointXYZ > ICPXYZ;

const string ICP::KEY_INPUT_POINT_CLOUD_SRC     = "source";
const string ICP::KEY_INPUT_POINT_CLOUD_TARGET  = "target";
const string ICP::KEY_OUTPUT_CONVERGENCE        = "convergence";
const string ICP::KEY_OUTPUT_SCORE              = "score";
const string ICP::KEY_OUTPUT_TRANSFORMATION     = "transformation";

/** @todo why does define guard lead to undefined reference error?
 */
//#ifdef __WITH_GTEST
#include <boost/assign/list_of.hpp>
template <>
elm::MapIONames LayerAttr_<ICP>::io_pairs = boost::assign::map_list_of
        ELM_ADD_INPUT_PAIR(ICP::KEY_INPUT_POINT_CLOUD_SRC)
        ELM_ADD_INPUT_PAIR(ICP::KEY_INPUT_POINT_CLOUD_TARGET)
        ELM_ADD_OUTPUT_PAIR(ICP::KEY_OUTPUT_CONVERGENCE)
        ELM_ADD_OUTPUT_PAIR(ICP::KEY_OUTPUT_SCORE)
        ELM_ADD_OUTPUT_PAIR(ICP::KEY_OUTPUT_TRANSFORMATION)
        ;
//#endif

ICP::ICP()
{
}

ICP::ICP(const LayerConfig &cfg)
{
}

void ICP::Clear()
{

}

void ICP::Reconfigure(const LayerConfig &cfg)
{

}

void ICP::Reset()
{

}

void ICP::Reset(const LayerConfig &cfg)
{

}

void ICP::InputNames(const LayerIONames &io)
{
    name_src_cloud_     = io.Input(KEY_INPUT_POINT_CLOUD_SRC);
    name_target_cloud_  = io.Input(KEY_INPUT_POINT_CLOUD_TARGET);
}

void ICP::OutputNames(const LayerIONames &io)
{
    name_convergence_   = io.Output(KEY_OUTPUT_CONVERGENCE);
    name_score_         = io.Output(KEY_OUTPUT_SCORE);
    name_transf_        = io.Output(KEY_OUTPUT_TRANSFORMATION);
}

void ICP::Activate(const Signal &signal)
{
    CloudXYZPtr cloud_src = signal.MostRecent(name_src_cloud_).get<CloudXYZPtr>();
    CloudXYZPtr cloud_target = signal.MostRecent(name_target_cloud_).get<CloudXYZPtr>();

    ICPXYZ icp;
    icp.setInputSource(cloud_src);
    icp.setInputTarget(cloud_target);

    CloudXYZ fin;
    icp.align(fin);

    has_conveged_ = icp.hasConverged();
    fitness_score_ = static_cast<float>(icp.getFitnessScore());
    eigen2cv(icp.getFinalTransformation(), transformation_);
}

void ICP::Response(Signal &signal)
{
    signal.Append(name_convergence_, Mat1f(1, 1, has_conveged_? 1.f : 0.f));
    signal.Append(name_score_, Mat1f(1, 1, fitness_score_));

    signal.Append(name_transf_, transformation_);
}

#endif // __WITH_PCL

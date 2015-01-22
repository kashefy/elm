/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, Elm Library Project
// 3-clause BSD License
//
//M*/
#include "elm/layers/icp.h"

#ifdef __WITH_PCL   // the layer is otherwise implemented as unsupported

#include <pcl/registration/icp.h>

#include <opencv2/core/eigen.hpp> // for eigen2cv(), must be preceeded definitio of Eigen either PCL or #include <Eigen/Dense>

#include "elm/core/exception.h"
#include "elm/core/layerconfig.h"
#include "elm/core/signal.h"

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

void ICP::IONames(const LayerIONames &io)
{
    // input names
    name_src_cloud_     = io.Input(KEY_INPUT_POINT_CLOUD_SRC);
    name_target_cloud_  = io.Input(KEY_INPUT_POINT_CLOUD_TARGET);

    // output names
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

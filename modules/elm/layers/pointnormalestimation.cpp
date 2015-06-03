/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/layers/pointnormalestimation.h"

#ifdef __WITH_PCL   // the layer is otherwise implemented as unsupported

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>

#include "elm/core/exception.h"
#include "elm/core/featuredata.h"
#include "elm/core/layerconfig.h"
#include "elm/core/signal.h"
#include "elm/ts/layerattr_.h"

using namespace std;
using namespace cv;
using namespace pcl;
using namespace elm;

// initialize paramter keys
const string PointNormalEstimation::PARAM_K_SEARCH        = "k_search";

// initialize I/O names
const string PointNormalEstimation::KEY_OUTPUT_POINT_CLOUD  = "cloud_out";

#include <boost/assign/list_of.hpp>
template <>
elm::MapIONames LayerAttr_<PointNormalEstimation>::io_pairs = boost::assign::map_list_of
        ELM_ADD_INPUT_PAIR(detail::BASE_SINGLE_INPUT_FEATURE_LAYER__KEY_INPUT_STIMULUS)
        ELM_ADD_OUTPUT_PAIR(PointNormalEstimation::KEY_OUTPUT_POINT_CLOUD)
        ;

PointNormalEstimation::PointNormalEstimation()
    : base_SingleInputFeatureLayer()
{
}

void PointNormalEstimation::Clear()
{
}

void PointNormalEstimation::Reconfigure(const LayerConfig &cfg)
{
    PTree p = cfg.Params();

    k_search_ = p.get<int>(PARAM_K_SEARCH);
}

void PointNormalEstimation::OutputNames(const LayerOutputNames &io) {

    name_dst_cloud_  = io.Output(KEY_OUTPUT_POINT_CLOUD);
}

void PointNormalEstimation::Activate(const Signal &signal)
{
    CloudXYZPtr cld_in = signal.MostRecent(name_input_).get<CloudXYZPtr>();

    if(cld_in->empty()) {

        ELM_THROW_BAD_DIMS("Cannot Activate normal estimation layer with empty input point cloud.");
    }

    // Normal estimation
    NormalEstimation<PointXYZ, Normal> norml_est;
    dst_cloud_.reset(new CloudNormal);
    search::KdTree<PointXYZ>::Ptr tree(new search::KdTree<PointXYZ>);

    tree->setInputCloud(cld_in);
    norml_est.setInputCloud(cld_in);
    norml_est.setSearchMethod(tree);
    norml_est.setKSearch(k_search_);
    norml_est.compute(*dst_cloud_);
    // normals should now contain the point normals + surface curvatures
}

void PointNormalEstimation::Response(Signal &signal)
{
    signal.Append(name_dst_cloud_, dst_cloud_);
}

#endif // __WITH_PCL

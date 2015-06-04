/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
/** @file Implement greedy projection triangulation
 */
#include "elm/layers/triangulation.h"

#ifdef __WITH_PCL   // the layer is otherwise implemented as unsupported

#include <pcl/kdtree/kdtree_flann.h>
//#include <opencv2/core/eigen.hpp>   // for eigen2cv(), must be preceeded definitio of Eigen either PCL or #include <Eigen/Dense>

#include "elm/core/defs.h"          // ELM_PI2
#include "elm/core/exception.h"
#include "elm/core/featuredata.h"
#include "elm/core/graph/adjacency.h"
#include "elm/core/inputname.h"
#include "elm/core/layerconfig.h"
#include "elm/core/signal.h"
#include "elm/ts/layerattr_.h"

using namespace std;
using namespace cv;
using namespace pcl;
using namespace elm;

extern template class pcl::PointCloud<pcl::PointXYZ >;
extern template class pcl::PointCloud<pcl::PointNormal >;

extern template class boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ > >;
extern template class boost::shared_ptr<pcl::PointCloud<pcl::PointNormal > >;

// initialize paramter keys
const string Triangulation::PARAM_SEARCH_RADIUS        = "radius";
const string Triangulation::PARAM_MU                   = "mu";
const string Triangulation::PARAM_MAX_NN               = "max_nn";
const string Triangulation::PARAM_MAX_SURFACE_ANGLE    = "max_surface_angle";
const string Triangulation::PARAM_MIN_ANGLE            = "min_angle";
const string Triangulation::PARAM_MAX_ANGLE            = "max_angle";
const string Triangulation::PARAM_IS_NORMAL_CONSISTENT = "normal_consistency";

// initialize default float paramter values
const float Triangulation::DEFAULT_SEARCH_RADIUS            = 0.025f;
const float Triangulation::DEFAULT_MU                       = 2.5f;
const float Triangulation::DEFAULT_MAX_SURFACE_ANGLE        = ELM_PI2 / 2.f;
const float Triangulation::DEFAULT_MIN_ANGLE                = CV_PI / 18.f;
const float Triangulation::DEFAULT_MAX_ANGLE                = 2 * CV_PI / 3.f;
const int   Triangulation::DEFAULT_MAX_NN                   = 100;  ///< maximum neighrest neighbors
const bool  Triangulation::DEFAULT_IS_NORMAL_CONSISTENCY    = false;

// initialize I/O names
const string Triangulation::KEY_INPUT_CLOUD_POINT_NORMAL    = "cloud";
const string Triangulation::KEY_OUTPUT_VERTICES             = "vertices";
const string Triangulation::KEY_OUTPUT_OPT_ADJACENCY        = "adjacency";

#include <boost/assign/list_of.hpp>
template <>
elm::MapIONames LayerAttr_<Triangulation>::io_pairs = boost::assign::map_list_of
        ELM_ADD_INPUT_PAIR(Triangulation::KEY_INPUT_CLOUD_POINT_NORMAL)
        ELM_ADD_OUTPUT_PAIR(Triangulation::KEY_OUTPUT_VERTICES)
        ;

Triangulation::Triangulation()
    : base_Layer()
{
}

void Triangulation::Clear()
{
    gp3 = pcl::GreedyProjectionTriangulation<pcl::PointNormal>();
}

void Triangulation::Reconfigure(const LayerConfig &cfg)
{
    PTree p = cfg.Params();

    float search_radius     = p.get<float>(PARAM_SEARCH_RADIUS, DEFAULT_SEARCH_RADIUS);

    float mu                = p.get<float>( PARAM_MU,     DEFAULT_MU);
    int max_nn              = p.get<int>(   PARAM_MAX_NN, DEFAULT_MAX_NN);
    float max_surface_angle = p.get<float>( PARAM_MAX_SURFACE_ANGLE, DEFAULT_MAX_SURFACE_ANGLE);
    float min_angle         = p.get<float>( PARAM_MIN_ANGLE, DEFAULT_MIN_ANGLE);
    float max_angle         = p.get<float>( PARAM_MAX_ANGLE, DEFAULT_MAX_ANGLE);
    bool is_normal_consistency = p.get<bool>(PARAM_IS_NORMAL_CONSISTENT, DEFAULT_IS_NORMAL_CONSISTENCY);

    // Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius(search_radius);

    // Set typical values for the parameters
    gp3.setMu(mu);
    gp3.setMaximumNearestNeighbors(max_nn);
    gp3.setMaximumSurfaceAngle(max_surface_angle);
    gp3.setMinimumAngle(min_angle);
    gp3.setMaximumAngle(max_angle);
    gp3.setNormalConsistency(is_normal_consistency);
}

void Triangulation::InputNames(const LayerInputNames &io)
{
    name_src_cloud_ = io.Input(KEY_INPUT_CLOUD_POINT_NORMAL);
}

void Triangulation::OutputNames(const LayerOutputNames &io)
{
    name_vertices_  = io.Output(KEY_OUTPUT_VERTICES);
    name_opt_adj_   = io.OutputOpt(KEY_OUTPUT_OPT_ADJACENCY);
}

void Triangulation::Activate(const Signal &signal)
{
    CloudPtNrmlPtr in = signal.MostRecent(name_src_cloud_).get<CloudPtNrmlPtr>();
    if(in->empty()) {

        ELM_THROW_BAD_DIMS("Cannot Activate Triangulation layer with empty input point cloud.");
    }

    // Create search tree*
    search::KdTree<PointNormal>::Ptr tree2(new search::KdTree<PointNormal>);
    tree2->setInputCloud(in);

    gp3.setInputCloud(in);
    gp3.setSearchMethod(tree2);

    // Get result
    Triangles vertices;
    gp3.reconstruct(vertices);

    vertices_ = VecVertices2Mat(vertices, false);

    if(name_opt_adj_) {

        CloudXYZPtr cld_xyz(new CloudXYZ);
        copyPointCloud(*in, *cld_xyz);

        TriangulatedCloudToAdjacency(cld_xyz, vertices, adj_);
    }
}

void Triangulation::Response(Signal &signal)
{
    // Additional vertex information
//    vector<int> parts = gp3.getPartIDs();
//    vector<int> states = gp3.getPointStates();

    signal.Append(name_vertices_, vertices_);

    if(name_opt_adj_) {

        signal.Append(name_opt_adj_.get(), adj_);
    }
}

#endif // __WITH_PCL

/** @file Implement greedy projection triangulation
 */
#include "layers/triangulation.h"

#ifdef __WITH_PCL   // the layer is otherwise implemented as unsupported

#include <iostream>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <opencv2/core/eigen.hpp> // for eigen2cv(), must be preceeded definitio of Eigen either PCL or #include <Eigen/Dense>

#include "core/defs.h"
#include "core/exception.h"
#include "core/layerconfig.h"
#include "core/mat_utils.h"
#include "core/pcl_utils.h"
#include "core/signal.h"

using namespace std;
using namespace cv;
using namespace pcl;
using namespace sem;

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
const float Triangulation::DEFAULT_MAX_SURFACE_ANGLE        = SEM_PI2 / 2.f;
const float Triangulation::DEFAULT_MIN_ANGLE                = CV_PI / 18.f;
const float Triangulation::DEFAULT_MAX_ANGLE                = 2 * CV_PI / 3.f;
const int   Triangulation::DEFAULT_MAX_NN                   = 100;  ///< maximum neighrest neighbors
const bool  Triangulation::DEFAULT_IS_NORMAL_CONSISTENCY    = false;

// initialize I/O names
const string Triangulation::KEY_INPUT_POINT_CLOUD   = "cloud";
const string Triangulation::KEY_OUTPUT_VERTICES     = "vertices";

Triangulation::Triangulation()
    : base_Layer()
{
    Reset();
}

Triangulation::Triangulation(const LayerConfig &cfg)
    : base_Layer(cfg)
{
    Reset(cfg);
}

void Triangulation::Clear()
{
    gp3 = pcl::GreedyProjectionTriangulation<pcl::PointNormal>();
}

void Triangulation::Reset()
{
    Reset(LayerConfig());
}

void Triangulation::Reset(const LayerConfig &cfg)
{
    Reconfigure(cfg);
}

void Triangulation::Reconfigure(const LayerConfig &cfg)
{
    PTree p = cfg.Params();

    float search_radius     = p.get<float>(PARAM_SEARCH_RADIUS, DEFAULT_SEARCH_RADIUS);

    float mu                = p.get<float>( PARAM_MU,     DEFAULT_MU);
    int max_nn              = p.get<int>(   PARAM_MAX_NN, DEFAULT_MAX_NN);
    float max_surface_angle = p.get<float>( PARAM_MAX_SURFACE_ANGLE, DEFAULT_MAX_SURFACE_ANGLE);
    float min_angle         = p.get<float>(PARAM_MIN_ANGLE, DEFAULT_MIN_ANGLE);
    float max_angle         = p.get<float>(PARAM_MAX_ANGLE, DEFAULT_MAX_ANGLE);
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

void Triangulation::IONames(const LayerIONames &io)
{
    name_src_cloud_ = io.Input(KEY_INPUT_POINT_CLOUD);
    name_vertices_  = io.Output(KEY_OUTPUT_VERTICES);
}

void Triangulation::Activate(const Signal &signal)
{
    Mat1f cld_in_mat = signal.MostRecent(name_src_cloud_);
    if(cld_in_mat.empty()) {

        SEM_THROW_BAD_DIMS("Cannot Activate Triangulation layer with empty input point cloud.");
    }

    CloudXYZ::Ptr cld_in = Mat2PointCloud(cld_in_mat);

    // Normal estimation*
    NormalEstimation<PointXYZ, Normal> n;
    PointCloud<Normal>::Ptr normals(new PointCloud<Normal>);
    search::KdTree<PointXYZ>::Ptr tree(new search::KdTree<PointXYZ>);

    tree->setInputCloud(cld_in);
    n.setInputCloud(cld_in);
    n.setSearchMethod(tree);
    n.setKSearch(20);
    n.compute(*normals);
    //* normals should not contain the point normals + surface curvatures

    // Concatenate the XYZ and normal fields*
    PointCloud<PointNormal>::Ptr cloud_with_normals(new PointCloud<PointNormal>);
    concatenateFields(*cld_in, *normals, *cloud_with_normals);
    //* cloud_with_normals = cloud + normals

    // Create search tree*
    search::KdTree<PointNormal>::Ptr tree2(new search::KdTree<PointNormal>);
    tree2->setInputCloud(cloud_with_normals);

    // Get result
    gp3.setInputCloud(cloud_with_normals);
    gp3.setSearchMethod(tree2);

    // PolygonMesh triangles;
    // gp3.reconstruct(triangles);

    std::vector<Vertices > vertices;
    gp3.reconstruct(vertices);

    vertices_ = VecVertices2Mat(vertices, true);
}

void Triangulation::Response(Signal &signal)
{
    // Additional vertex information
//    vector<int> parts = gp3.getPartIDs();
//    vector<int> states = gp3.getPointStates();

    signal.Append(name_vertices_, vertices_);
}

#endif // __WITH_PCL

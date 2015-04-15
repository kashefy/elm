/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/layers/triangulation.h"

#ifdef __WITH_PCL // test normally

#include <boost/filesystem.hpp>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>

#include <pcl/io/vtk_io.h>

#include "elm/core/exception.h"
#include "elm/core/layerconfig.h"
#include "elm/core/pcl/vertices.h"
#include "elm/core/signal.h"
#include "elm/layers/layerfactory.h"
#include "elm/ts/ts.h"
#include "elm/ts/layer_assertions.h"

using namespace std;
namespace bfs=boost::filesystem;
using namespace cv;
using namespace pcl;
using namespace elm;

namespace {

ELM_INSTANTIATE_LAYER_TYPED_TEST_CASE_P(Triangulation);

const bfs::path TEST_DIR("testdata");
const bfs::path TEST_PATH_PCD = TEST_DIR/"bun0.pcd";

// Names for I/O
const string NAME_INPUT_POINT_CLOUD = "in";   ///< name of input point cloud
const string NAME_OUTPUT_VERTICES   = "v";    ///< name of output vertices
const string NAME_OUTPUT_OPT_ADJ    = "adj";  ///< name of optional output adjacency matrix

class TriangulationInitTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        cfg_ = LayerConfig();
        cfg_.Params(params_);

        io_names_.Input(Triangulation::KEY_INPUT_POINT_CLOUD,   NAME_INPUT_POINT_CLOUD);
        io_names_.Output(Triangulation::KEY_OUTPUT_VERTICES,     NAME_OUTPUT_VERTICES);
    }

    virtual void TearDown()
    {
        params_.clear();
        io_names_ = LayerIONames();
    }

    // members
    PTree params_;
    LayerConfig cfg_;
    LayerIONames io_names_;
};

TEST_F(TriangulationInitTest, Constructor)
{
    EXPECT_NO_THROW(Triangulation to(cfg_));
}

TEST_F(TriangulationInitTest, MissingParams)
{
    cfg_.Params(PTree());
    EXPECT_NO_THROW(Triangulation to(cfg_));

    Triangulation to;
    EXPECT_NO_THROW(to.Reset(cfg_));
    EXPECT_NO_THROW(to.Reconfigure(cfg_));
}

TEST_F(TriangulationInitTest, CreateWithFactory)
{
    shared_ptr<base_Layer> to_ptr = LayerFactory::CreateShared("Triangulation", cfg_, io_names_);

    EXPECT_TRUE(bool(to_ptr));
}

/**
 * @brief Test the live methods assuming a sucessful initialization
 */
class TriangulationTest : public TriangulationInitTest
{
protected:
    virtual void SetUp()
    {
        TriangulationInitTest::SetUp();

        // Load input file into a PointCloudXYZ
        cloud_in_.reset(new CloudXYZ);
        PCLPointCloud2 cloud_blob;
        pcl::io::loadPCDFile(TEST_PATH_PCD.string(), cloud_blob);
        fromPCLPointCloud2 (cloud_blob, *cloud_in_);

        to_ = LayerFactory::CreateShared("Triangulation", cfg_, io_names_);

        sig_.Append(NAME_INPUT_POINT_CLOUD, PointCloud2Mat_<PointXYZ>(cloud_in_));
    }

    virtual void TearDown()
    {
        TriangulationInitTest::TearDown();
        sig_.Clear();
    }

    shared_ptr<base_Layer> to_; ///< pointer to test object
    Signal sig_;
    CloudXYZPtr cloud_in_;
};

TEST_F(TriangulationTest, ActivateEmptyInput)
{
    sig_.Append(NAME_INPUT_POINT_CLOUD, Mat1f());
    EXPECT_THROW(to_->Activate(sig_), ExceptionBadDims);
}

TEST_F(TriangulationTest, ActivateAndResponse)
{
    // Normal estimation*
    NormalEstimation<PointXYZ, Normal> n;
    PointCloud<Normal>::Ptr normals(new PointCloud<Normal>);
    search::KdTree<PointXYZ>::Ptr tree(new search::KdTree<PointXYZ>);

    tree->setInputCloud(cloud_in_);
    n.setInputCloud(cloud_in_);
    n.setSearchMethod(tree);
    n.setKSearch(20);
    n.compute(*normals);
    //* normals should not contain the point normals + surface curvatures

    // Concatenate the XYZ and normal fields*
    PointCloud<PointNormal>::Ptr cloud_with_normals(new PointCloud<PointNormal>);
    concatenateFields(*cloud_in_, *normals, *cloud_with_normals);
    //* cloud_with_normals = cloud + normals

    // Create search tree*
    search::KdTree<PointNormal>::Ptr tree2(new search::KdTree<PointNormal>);
    tree2->setInputCloud(cloud_with_normals);

    // Initialize objects
    GreedyProjectionTriangulation<PointNormal> gp3;

    // Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius(Triangulation::DEFAULT_SEARCH_RADIUS);

    // Set typical values for the parameters
    gp3.setMu(                      Triangulation::DEFAULT_MU);
    gp3.setMaximumNearestNeighbors( Triangulation::DEFAULT_MAX_NN);
    gp3.setMaximumSurfaceAngle(     Triangulation::DEFAULT_MAX_SURFACE_ANGLE);
    gp3.setMinimumAngle(            Triangulation::DEFAULT_MIN_ANGLE);
    gp3.setMaximumAngle(            Triangulation::DEFAULT_MAX_ANGLE);
    gp3.setNormalConsistency(       Triangulation::DEFAULT_IS_NORMAL_CONSISTENCY);

    // Get result
    gp3.setInputCloud(cloud_with_normals);
    gp3.setSearchMethod(tree2);

    //PolygonMesh triangles;
    //gp3.reconstruct(triangles);

    vector<Vertices > vertices_vec;
    gp3.reconstruct(vertices_vec);

    // Additional vertex information
//    vector<int> parts = gp3.getPartIDs();
//    vector<int> states = gp3.getPointStates();

//    cout<<triangles<<endl;
    // Finish
    //pcl::io::saveVTKFile ("mesh.vtk", triangles);

    to_->Activate(sig_);

    EXPECT_FALSE(sig_.Exists(NAME_OUTPUT_VERTICES)) << "Output feature already exists, better clear signal first.";

    to_->Response(sig_);

    EXPECT_TRUE(sig_.Exists(NAME_OUTPUT_VERTICES)) << "Output feature is missing.";

    Mat1f vertices_mat = sig_.MostRecent(NAME_OUTPUT_VERTICES);

    int nb_vertices = static_cast<int>(vertices_vec.size());
    int sz_vertex = static_cast<int>(vertices_vec[0].vertices.size());

    EXPECT_MAT_DIMS_EQ(vertices_mat, Size2i(sz_vertex, nb_vertices)) << "Dimensions do not match.";

    int k=0;
    for(int i=0; i<nb_vertices; i++) {

        vector<uint32_t> tmp = vertices_vec[i].vertices;
        for(int j=0; j<sz_vertex; j++) {

            EXPECT_FLOAT_EQ(static_cast<float>(tmp[j]), vertices_mat(k++)) << "Vertex mismatch.";
        }
    }
}

class TriangulationAdjacencyTest : public TriangulationTest
{
protected:
    virtual void SetUp()
    {
        io_names_.Output(Triangulation::KEY_OUTPUT_OPT_ADJACENCY, NAME_OUTPUT_OPT_ADJ);
        TriangulationTest::SetUp();
    }
};

TEST_F(TriangulationAdjacencyTest, NoAdjacency)
{
    LayerIONames io_names;
    io_names.Input(Triangulation::KEY_INPUT_POINT_CLOUD,    NAME_INPUT_POINT_CLOUD);
    io_names.Output(Triangulation::KEY_OUTPUT_VERTICES,     NAME_OUTPUT_VERTICES);

    to_ = LayerFactory::CreateShared("Triangulation", cfg_, io_names);

    to_->Activate(sig_);
    to_->Response(sig_);

    EXPECT_FALSE(sig_.Exists(NAME_OUTPUT_OPT_ADJ));
}

TEST_F(TriangulationAdjacencyTest, Adjacency)
{
    to_->Activate(sig_);
    to_->Response(sig_);

    EXPECT_TRUE(sig_.Exists(NAME_OUTPUT_OPT_ADJ));

    Mat1f adj = sig_.MostRecent(NAME_OUTPUT_OPT_ADJ).get<Mat1f>();

    EXPECT_MAT_DIMS_EQ(adj, Size2i(cloud_in_->size(), cloud_in_->size())) << "Expecting no. of vertices to match no. of points in the cloud.";

    EXPECT_MAT_EQ(adj, adj.t()) << "Expecting symmetric adjacency matrix.";
}

} // annonymous namespace for test fixtures

#endif // __WITH_PCL

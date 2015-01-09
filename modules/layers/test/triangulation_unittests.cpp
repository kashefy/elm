#include "layers/triangulation.h"

#include <boost/filesystem.hpp>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>

#include <pcl/io/vtk_io.h>

#include "core/exception.h"
#include "core/layerconfig.h"
#include "core/pcl_utils.h"
#include "core/signal.h"
#include "layers/layerfactory.h"
#include "ts/ts.h"
#include "ts/layer_assertions.h"

using namespace std;
namespace bfs=boost::filesystem;
using namespace cv;
using namespace pcl;
using namespace sem;

namespace {

#ifdef __WITH_PCL // test normally

const bfs::path TEST_DIR("testdata");
const bfs::path TEST_PATH_PCD = TEST_DIR/"bun0.pcd";

const string NAME_INPUT_POINT_CLOUD = "in";    ///< name of input point cloud

class TriangulationInitTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        cfg_ = LayerConfig();
        cfg_.Params(params_);
        io_names_ = LayerIONames();

        io_names_.Input(Triangulation::KEY_INPUT_POINT_CLOUD, NAME_INPUT_POINT_CLOUD);
    }

    virtual void TearDown()
    {
        params_.clear();
    }

    // members
    PTree params_;
    LayerConfig cfg_;
    LayerIONames io_names_;
};

TEST_F(TriangulationInitTest, Constructor)
{
    EXPECT_NO_THROW(Triangulation to);
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

TEST_F(TriangulationInitTest, MissingRequiredIONames)
{
    shared_ptr<base_Layer> to_ptr(new Triangulation()); // pointer to test object

    EXPECT_THROW(to_ptr->IONames(LayerIONames()), ExceptionKeyError);

    map<string, pair<bool, string> > io_pairs; // false for input, true for output
    io_pairs[Triangulation::KEY_INPUT_POINT_CLOUD] = make_pair(0, NAME_INPUT_POINT_CLOUD);

    ValidateRequiredIONames(io_pairs, to_ptr);
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
        cloud_in_.reset(new PointCloudXYZ);
        PCLPointCloud2 cloud_blob;
        pcl::io::loadPCDFile(TEST_PATH_PCD.string(), cloud_blob);
        fromPCLPointCloud2 (cloud_blob, *cloud_in_);

        to_ = LayerFactory::CreateShared("Triangulation", cfg_, io_names_);
    }

    virtual void TearDown()
    {
        TriangulationInitTest::TearDown();
    }

    shared_ptr<base_Layer> to_; ///< pointer to test object
    Signal sig_;
    PointCloudXYZ::Ptr cloud_in_;
};


TEST_F(TriangulationTest, FOO)
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
    PolygonMesh triangles;

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
    gp3.reconstruct(triangles);

    // Additional vertex information
    vector<int> parts = gp3.getPartIDs();
    vector<int> states = gp3.getPointStates();

    cout<<triangles<<endl;
    // Finish
    //pcl::io::saveVTKFile ("mesh.vtk", triangles);
}

#else // __WITH_PCL
    #warning "Skipping building Triangulation layer unit tests"
#endif // __WITH_PCL

} // annonymous namespace for test fixtures

